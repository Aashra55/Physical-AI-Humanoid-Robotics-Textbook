from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Union
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance
from core.settings import settings
from sentence_transformers import SentenceTransformer
import psycopg2, os, hashlib, math

app = FastAPI()

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---------- DATABASE ----------
DB_URL = settings.NEON_DB_URL

# ---------- QDRANT ----------
QDRANT_URL = settings.QDRANT_URL
COLLECTION_NAME = settings.QDRANT_COLLECTION_NAME
QDRANT_API_KEY = settings.QDRANT_API_KEY

client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY
)

# Load real embedding model
print("DEBUG: Loading SentenceTransformer model (all-MiniLM-L6-v2)...")
model = SentenceTransformer("all-MiniLM-L6-v2")

# ---------- ENSURE COLLECTION ----------
def ensure_collection():
    try:
        if client.collection_exists(collection_name=COLLECTION_NAME):
            return
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=384, distance=Distance.COSINE) # 384 for all-MiniLM-L6-v2
        )
    except Exception as e:
        print(f"DEBUG ERROR in ensure_collection: {e}")

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
def get_embedding(text: str) -> List[float]:
    return model.encode(text).tolist()


def get_conn():
    return psycopg2.connect(DB_URL)

def is_greeting(text: str) -> bool:
    greetings = ["hi", "hello", "hey", "assalam", "salaam", "greetings", "good morning", "good afternoon"]
    text = text.lower().strip()
    return any(text.startswith(g) or text == g for g in greetings)

# ---------- CHAT ENDPOINT ----------
@app.post("/chat")
async def chat(query: Query):
    """Frontend-friendly endpoint for chatbot"""
    print(f"DEBUG: Received request at /chat: {query.question}")
    
    # 1. Handle Greetings Generously
    if is_greeting(query.question):
        return {"results": ["Hello! I am your Physical AI & Humanoid Robotics assistant. How can I help you explore the textbook today?"]}

    # 2. Perform Semantic Search
    try:
        vec = get_embedding(query.question)
        print(f"DEBUG: Searching Qdrant collection: {COLLECTION_NAME}")

        result = client.query_points(
            collection_name=COLLECTION_NAME,
            query=vec,
            limit=query.top_k
        ).points
        
        print(f"DEBUG: Qdrant search returned {len(result)} hits.")

        if not result:
            return {"results": ["I couldn't find specific information about that in the book, but I'm happy to help with other topics like ROS 2, Gazebo, or Isaac Sim!"]}

        hits = []
        # Try to get content from PostgreSQL if payload is just metadata
        conn = get_conn()
        cur = conn.cursor()
        for hit in result:
            doc_id = hit.id
            cur.execute("SELECT content FROM documents WHERE id = %s", (str(doc_id),))
            row = cur.fetchone()
            if row:
                hits.append(row[0])
            elif hit.payload and "text" in hit.payload:
                hits.append(hit.payload["text"])
        
        cur.close()
        conn.close()

        if not hits:
            return {"results": ["I found some matches but couldn't retrieve the text content. Please try indexing again."]}

        return {"results": hits}

    except Exception as e:
        print(f"DEBUG ERROR: Chat failed: {e}")
        return {"results": [f"Sorry, I encountered an error while searching: {str(e)}"]}


# ---------- HEALTH ----------
@app.get("/")
def health():
    return {"status": "running"}
