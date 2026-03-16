import logging
import os
import glob
import markdown
import json
import uuid
from bs4 import BeautifulSoup
import qdrant_client
from qdrant_client.models import PointStruct, VectorParams, Distance
from sentence_transformers import SentenceTransformer

from .settings import settings
from .db import get_db_connection, get_qdrant_client

# Configure basic logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def setup_databases():
    """
    Sets up the Postgres table and Qdrant collection.
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
        correct_vector_size = 384

        try:
            collection_info = qdrant_cli.get_collection(collection_name=collection_name)
            current_vector_size = collection_info.config.params.vectors.size
            if current_vector_size != correct_vector_size:
                logger.info(f"Recreating collection {collection_name} for correct size.")
                qdrant_cli.delete_collection(collection_name=collection_name)
                qdrant_cli.create_collection(
                    collection_name=collection_name,
                    vectors_config=VectorParams(size=correct_vector_size, distance=Distance.COSINE),
                )
        except Exception:
            logger.info(f"Creating new collection {collection_name}.")
            qdrant_cli.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=correct_vector_size, distance=Distance.COSINE),
            )
    except Exception as e:
        logger.error(f"Error setting up databases: {e}")


def chunk_text(text: str, chunk_size: int = 1000, overlap_size: int = 100):
    """
    Splits text into chunks of a specified size with overlap.
    Increased chunk size for better context.
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
    setup_databases()
    logger.info("Indexing documents...")
    total_chunks_indexed = 0
    try:
        embedding_model = SentenceTransformer("all-MiniLM-L6-v2")
        conn = get_db_connection()
        qdrant_cli = get_qdrant_client()
        collection_name = settings.QDRANT_COLLECTION_NAME

        cur = conn.cursor()

        docs_path = os.path.abspath(settings.DOCS_PATH)
        markdown_files = glob.glob(os.path.join(docs_path, "**", "*.md"), recursive=True)
        logger.info(f"Found {len(markdown_files)} Markdown files.")

        for md_file in markdown_files:
            try:
                with open(md_file, "r", encoding="utf-8") as f:
                    content = f.read()

                html = markdown.markdown(content)
                soup = BeautifulSoup(html, "html.parser")
                text = soup.get_text()

                chunks = chunk_text(text)
                source = os.path.relpath(md_file, docs_path)

                for i, chunk in enumerate(chunks):
                    doc_id = uuid.uuid4()
                    embedding = embedding_model.encode(chunk).tolist()

                    cur.execute(
                        "INSERT INTO documents (id, content, source, chunk_num) VALUES (%s, %s, %s, %s)",
                        (str(doc_id), chunk, source, i)
                    )

                    # CRITICAL: Store the actual text in the payload
                    qdrant_cli.upsert(
                        collection_name=collection_name,
                        wait=True,
                        points=[
                            PointStruct(
                                id=str(doc_id),
                                vector=embedding,
                                payload={"source": source, "chunk_num": i, "content": chunk},
                            )
                        ],
                    )
                    total_chunks_indexed += 1
            except Exception as file_e:
                logger.error(f"Error processing file {md_file}: {file_e}")

        conn.commit()
        cur.close()
        conn.close()
        logger.info(f"Successfully indexed {total_chunks_indexed} chunks.")
    except Exception as e:
        logger.error(f"Error indexing documents: {e}")

if __name__ == "__main__":
    index_documents()
