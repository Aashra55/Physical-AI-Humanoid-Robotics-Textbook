import logging
import os
import glob
import markdown
import json
import uuid
from bs4 import BeautifulSoup
import qdrant_client
import psycopg2
from sentence_transformers import SentenceTransformer

from .settings import settings

# --- 1. Database and Client Initialization ---

def get_db_connection():
    """Establishes a connection to the Neon Postgres database."""
    conn = psycopg2.connect(settings.NEON_DB_URL)
    return conn

def get_qdrant_client():
    """Initializes and returns the Qdrant client."""
    client = qdrant_client.QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )
    return client

def setup_databases():
    """
    Sets up the Postgres table and Qdrant collection. If the Qdrant collection
    exists with the wrong vector size, it is recreated.
    """
    logging.info("Setting up databases...")
    try:
        # --- Setup Postgres ---
        conn = get_db_connection()
        cur = conn.cursor()
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS documents (
                id UUID PRIMARY KEY,
                content TEXT,
                source VARCHAR(255),
                chunk_num INTEGER,
                metadata JSONB DEFAULT '{}'
            );
            """
        )
        conn.commit()
        cur.close()
        conn.close()
        logging.info("Postgres 'documents' table ensured.")

        # --- Setup Qdrant ---
        qdrant_cli = get_qdrant_client()
        collection_name = settings.QDRANT_COLLECTION_NAME
        correct_vector_size = 384  # Vector size for all-MiniLM-L6-v2

        try:
            collection_info = qdrant_cli.get_collection(collection_name=collection_name)
            current_vector_size = collection_info.config.params.vectors.size
            
            if current_vector_size != correct_vector_size:
                logging.info(f"Qdrant collection '{collection_name}' exists with the wrong vector size ({current_vector_size}). Deleting and recreating.")
                qdrant_cli.delete_collection(collection_name=collection_name)
                qdrant_cli.create_collection(
                    collection_name=collection_name,
                    vectors_config=qdrant_client.models.VectorParams(size=correct_vector_size, distance=qdrant_client.models.Distance.COSINE),
                )
                logging.info(f"Qdrant collection '{collection_name}' recreated with correct vector size ({correct_vector_size}).")
            else:
                logging.info(f"Qdrant collection '{collection_name}' already exists with the correct vector size ({correct_vector_size}).")

        except Exception: # If collection doesn't exist at all
            logging.info(f"Qdrant collection '{collection_name}' does not exist. Creating it.")
            qdrant_cli.create_collection(
                collection_name=collection_name,
                vectors_config=qdrant_client.models.VectorParams(size=correct_vector_size, distance=qdrant_client.models.Distance.COSINE),
            )
            logging.info(f"Qdrant collection '{collection_name}' created with vector size ({correct_vector_size}).")
    except Exception as e:
        logging.error(f"Error setting up databases: {e}", exc_info=True)


def chunk_text(text: str, chunk_size: int = 200, overlap_size: int = 50):
    """
    Splits text into chunks of a specified size with overlap.
    A more sophisticated chunking strategy may be needed for production.
    TEMPORARILY REDUCED CHUNK_SIZE AND OVERLAP_SIZE FOR DEBUGGING.
    """
    if not text:
        return []

    chunks = []
    current_pos = 0
    while current_pos < len(text):
        end_pos = min(current_pos + chunk_size, len(text))
        chunk = text[current_pos:end_pos]
        chunks.append(chunk)
        if end_pos == len(text):
            break
        current_pos += chunk_size - overlap_size
        if current_pos < 0: # Handle cases where overlap_size > chunk_size, or very small chunks
            current_pos = 0
    return chunks

def index_documents():
    logging.info("Indexing documents...")
    total_chunks_indexed = 0
    try:
        embedding_model = SentenceTransformer("all-MiniLM-L6-v2")
        conn = get_db_connection()
        qdrant_cli = get_qdrant_client()
        collection_name = settings.QDRANT_COLLECTION_NAME

        cur = conn.cursor()

        docs_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "website", "docs"))
        markdown_files = glob.glob(os.path.join(docs_path, "**", "*.md"), recursive=True)
        logging.info(f"Found {len(markdown_files)} Markdown files in {docs_path}")

        for md_file in markdown_files:
            try:
                with open(md_file, "r", encoding="utf-8") as f:
                    content = f.read()

                html = markdown.markdown(content)
                soup = BeautifulSoup(html, "html.parser")
                text = soup.get_text()

                chunks = chunk_text(text)
                logging.info(f"Processing file: {os.path.relpath(md_file, docs_path)} - Text length: {len(text)}, Generated {len(chunks)} chunks.")

                source = os.path.relpath(md_file, docs_path)

                for i, chunk in enumerate(chunks):
                    doc_id = uuid.uuid4()
                    embedding = embedding_model.encode(chunk).tolist()

                    cur.execute(
                        """
                        INSERT INTO documents (id, content, source, chunk_num)
                        VALUES (%s, %s, %s, %s)
                        """,
                        (str(doc_id), chunk, source, i)
                    )

                    qdrant_cli.upsert(
                        collection_name=collection_name,
                        wait=True,
                        points=[
                            qdrant_client.models.PointStruct(
                                id=str(doc_id),
                                vector=embedding,
                                payload={"source": source, "chunk_num": i},
                            )
                        ],
                    )
                    total_chunks_indexed += 1
            except Exception as file_e:
                logging.error(f"Error processing file {os.path.relpath(md_file, docs_path)}: {file_e}", exc_info=True)

        conn.commit()
        cur.close()
        conn.close()
        logging.info(f"Documents indexed successfully. Total chunks indexed: {total_chunks_indexed}")
    except Exception as e:
        logging.error(f"Error indexing documents: {e}", exc_info=True)