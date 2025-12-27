from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Union
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance
import psycopg2, os, hashlib, math

app = FastAPI()

# ---------- DATABASE ----------
DB_URL = os.getenv(
    "NEON_DB_URL"
)

# ---------- QDRANT ----------
QDRANT_URL = os.getenv("QDRANT_URL")
COLLECTION_NAME = os.getenv("COLLECTION_NAME")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY
)


# ---------- ENSURE COLLECTION ----------
def ensure_collection():
    if client.collection_exists(collection_name=COLLECTION_NAME):
        # Optional: check vector size and distance if needed
        return
    client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=64, distance=Distance.COSINE)
    )

ensure_collection()


# ---------- MODELS ----------
class Document(BaseModel):
    id: Union[int, str]
    text: str
    vector: List[float] | None = None


class Query(BaseModel):
    question: str
    top_k: int = 3


# ---------- HELPERS ----------
def simple_embed(text: str) -> List[float]:
    h = hashlib.sha256(text.encode()).digest()
    nums = [b / 255 for b in h[:64]]
    norm = math.sqrt(sum(x*x for x in nums))
    return [x / (norm or 1) for x in nums]


def get_conn():
    return psycopg2.connect(DB_URL)


# ---------- UPSERT DOCUMENT ----------
@app.post("/upsert")
async def upsert_document(doc: Document):
    try:
        vec = doc.vector or simple_embed(doc.text)

        # Upsert to Qdrant
        client.upsert(
            collection_name=COLLECTION_NAME,
            points=[
                PointStruct(
                    id=str(doc.id),
                    vector=vec,
                    payload={"text": doc.text}
                )
            ],
        )

        # Upsert to PostgreSQL
        try:
            conn = get_conn()
            cur = conn.cursor()
            cur.execute(
                "CREATE TABLE IF NOT EXISTS documents (id TEXT PRIMARY KEY, text TEXT)"
            )
            cur.execute(
                "INSERT INTO documents (id, text) VALUES (%s, %s) ON CONFLICT (id) DO NOTHING",
                (str(doc.id), doc.text),
            )
            conn.commit()
            cur.close()
            conn.close()
        except Exception as db_err:
            print(f"PostgreSQL upsert failed: {db_err}")

        return {"status": "ok"}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Upsert failed: {e}")


# ---------- SEARCH DOCUMENTS ----------
@app.post("/search")
async def search_docs(query: Query):
    try:
        vec = simple_embed(query.question)

        result = client.search(
            collection_name=COLLECTION_NAME,
            query_vector=vec,
            limit=query.top_k
        )

        hits = [hit.payload.get("text") for hit in result]
        return {"results": hits}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Qdrant search failed: {e}")


# ---------- CHAT ENDPOINT ----------
@app.post("/chat")
async def chat(query: Query):
    """Frontend-friendly endpoint for chatbot"""
    return await search_docs(query)


# ---------- HEALTH ----------
@app.get("/")
def health():
    return {"status": "running"}
