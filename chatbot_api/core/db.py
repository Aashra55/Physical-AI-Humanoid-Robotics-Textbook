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

def get_document_content(db_conn, doc_ids: list):
    """Retrieves document content from Postgres for a list of UUIDs."""
    cur = db_conn.cursor()
    # Ensure doc_ids are strings for the IN clause
    str_doc_ids = [str(doc_id) for doc_id in doc_ids]
    # Use sql.In for safe parameter passing with lists
    query = f"SELECT content FROM documents WHERE id IN ({', '.join(['%s'] * len(str_doc_ids))});"
    cur.execute(query, str_doc_ids)
    results = cur.fetchall()
    cur.close()
    return [row[0] for row in results]
