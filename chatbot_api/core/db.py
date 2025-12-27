import psycopg2
import qdrant_client

from .settings import settings

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
