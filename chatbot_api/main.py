from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, Filter, FieldCondition, MatchValue
import psycopg2
import os

app = FastAPI()

# ---------- DATABASE CONFIG ----------
DB_URL = os.getenv(
    "DATABASE_URL",
    "postgresql://neondb_owner:npg_NlWTCJMZxO35@ep-twilight-hill-agk4goai-pooler.c-2.eu-central-1.aws.neon.tech/neondb?sslmode=require"
)

# ---------- QDRANT CONFIG ----------
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
COLLECTION_NAME = "documents"

client = QdrantClient(url=QDRANT_URL)

# ---------- MODELS ----------
class Document(BaseModel):
    id: int
    text: str
    vector: List[float]

class Query(BaseModel):
    query_vector: List[float]
    top_k: int = 3


# ---------- DB CONNECTION ----------
def get_conn():
    return psycopg2.connect(DB_URL)


# ---------- UPSERT ----------
@app.post("/upsert")
async def upsert_document(doc: Document):
    try:
        client.upsert(
            collection_name=COLLECTION_NAME,
            points=[PointStruct(id=doc.id, vector=doc.vector, payload={"text": doc.text})]
        )

        conn = get_conn()
        cur = conn.cursor()
        cur.execute(
            "INSERT INTO documents (id, text) VALUES (%s, %s) ON CONFLICT (id) DO NOTHING",
            (doc.id, doc.text)
        )
        conn.commit()
        cur.close()
        conn.close()

        return {"status": "ok"}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Upsert failed: {e}")


# ---------- SEARCH ----------
@app.post("/search")
async def search_docs(query: Query):
    try:
        result = client.query_points(
            collection_name=COLLECTION_NAME,
            query=query.query_vector,
            limit=query.top_k
        )

        hits = [hit.payload.get("text") for hit in result.points]
        return {"results": hits}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Qdrant search failed: {e}")


# ---------- HEALTH ----------
@app.get("/")
def health():
    return {"status": "running"}
