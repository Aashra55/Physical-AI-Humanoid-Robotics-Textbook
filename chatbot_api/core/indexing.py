import logging
import os
import glob
import markdown
import uuid
from bs4 import BeautifulSoup
import qdrant_client
from qdrant_client.models import PointStruct, VectorParams, Distance # Import Qdrant models directly
from sentence_transformers import SentenceTransformer

from .settings import settings
from .db import get_db_connection, get_qdrant_client

# --- Configure basic logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- 1. Database and Client Initialization ---

def setup_databases():
    """
    Sets up the Postgres table and Qdrant collection. If the Qdrant collection
    exists with the wrong vector size, it is recreated.
    """
    logger.info("Setting up databases...")
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
        logger.info("Postgres 'documents' table ensured.")

        # --- Setup Qdrant ---
        qdrant_cli = get_qdrant_client()
        collection_name = settings.QDRANT_COLLECTION_NAME
        correct_vector_size = 384  # Vector size for all-MiniLM-L6-v2 (used by fast-bge-small-en)

        # Explicitly delete collection to ensure a clean slate before creation/recreation
        try:
            qdrant_cli.delete_collection(collection_name=collection_name)
            logger.info(f"Existing Qdrant collection '{collection_name}' deleted for recreation.")
        except Exception as e:
            logger.info(f"Qdrant collection '{collection_name}' did not exist, no deletion needed (Error: {e}).")

        try:
            # After deletion, create it with the correct config
            qdrant_cli.create_collection(
                collection_name=collection_name,
                vectors_config={
                    "fast-bge-small-en": VectorParams(size=correct_vector_size, distance=Distance.COSINE)
                },
            )
            logger.info(f"Qdrant collection '{collection_name}' created with named vector size ({correct_vector_size}).")

        except Exception as e: # Catching broad exceptions if collection does not exist or for other issues
            logger.error(f"Error creating Qdrant collection '{collection_name}': {e}", exc_info=True)
            raise # Re-raise the exception as creation is critical

    except Exception as e:
        logger.error(f"Error setting up databases: {e}", exc_info=True)


def chunk_text(text: str, chunk_size: int = 200, overlap_size: int = 50):
    """
    Splits text into chunks of a specified size with overlap.
    A more sophisticated chunking strategy may be needed for production.
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
    return chunks

def index_documents():
    logger.info("Indexing documents...")
    total_chunks_indexed = 0
    try:
        embedding_model = SentenceTransformer("all-MiniLM-L6-v2") # Re-introduced
        conn = get_db_connection()
        qdrant_cli = get_qdrant_client()
        collection_name = settings.QDRANT_COLLECTION_NAME

        cur = conn.cursor()

        # Use the path from settings, ensuring it's absolute
        docs_path = os.path.abspath(settings.DOCS_PATH)
        markdown_files = glob.glob(os.path.join(docs_path, "**", "*.md"), recursive=True)
        logger.info(f"Found {len(markdown_files)} Markdown files in {docs_path}")

        for md_file in markdown_files:
            try:
                with open(md_file, "r", encoding="utf-8") as f:
                    content = f.read()

                html = markdown.markdown(content)
                soup = BeautifulSoup(html, "html.parser")
                text = soup.get_text()

                chunks = chunk_text(text)
                logger.info(f"Processing file: {os.path.relpath(md_file, docs_path)} - Text length: {len(text)}, Generated {len(chunks)} chunks.")

                source = os.path.relpath(md_file, docs_path)

                for i, chunk in enumerate(chunks):
                    doc_id = uuid.uuid4()
                    embedding = embedding_model.encode(chunk).tolist() # Re-introduced

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
                            PointStruct(
                                id=str(doc_id),
                                # Qdrant expects the computed vector for upsert
                                vector={
                                    "fast-bge-small-en": embedding # Pass the computed embedding here
                                },
                                payload={"source": source, "chunk_num": i, "content": chunk}, # Added 'content' to payload
                            )
                        ],
                    )
                    total_chunks_indexed += 1
            except Exception as file_e:
                logger.error(f"Error processing file {os.path.relpath(md_file, docs_path)}: {file_e}", exc_info=True)

        conn.commit()
        cur.close()
        conn.close()
        logger.info(f"Documents indexed successfully. Total chunks indexed: {total_chunks_indexed}")
    except Exception as e:
        logger.error(f"Error indexing documents: {e}", exc_info=True)


if __name__ == "__main__":
    setup_databases()
    index_documents()