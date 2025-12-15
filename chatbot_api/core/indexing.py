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
    print("Postgres 'documents' table ensured.")

    # --- Setup Qdrant ---
    qdrant_cli = get_qdrant_client()
    collection_name = settings.QDRANT_COLLECTION_NAME
    correct_vector_size = 384  # Vector size for all-MiniLM-L6-v2

    try:
        collection_info = qdrant_cli.get_collection(collection_name=collection_name)
        current_vector_size = collection_info.vectors_config.params.size
        
        if current_vector_size == correct_vector_size:
            print(f"Qdrant collection '{collection_name}' already exists with the correct vector size ({correct_vector_size}).")
        else:
            print(f"Qdrant collection '{collection_name}' exists with the wrong vector size ({current_vector_size}). Deleting and recreating.")
            qdrant_cli.delete_collection(collection_name=collection_name)
            qdrant_cli.create_collection(
                collection_name=collection_name,
                vectors_config=qdrant_client.models.VectorParams(size=correct_vector_size, distance=qdrant_client.models.Distance.COSINE),
            )
            print(f"Qdrant collection '{collection_name}' recreated with correct vector size ({correct_vector_size}).")

    except Exception: # If collection doesn't exist at all
        print(f"Qdrant collection '{collection_name}' does not exist. Creating it.")