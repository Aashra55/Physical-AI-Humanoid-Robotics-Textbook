# ===============================
# RAG Chatbot – FastAPI Backend
# ===============================

import os
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from dotenv import load_dotenv
from fastapi.middleware.cors import CORSMiddleware
from fastapi.concurrency import run_in_threadpool

from sentence_transformers import SentenceTransformer
import litellm

from core.settings import settings
from core.db import (
    get_db_connection,
    get_qdrant_client,
    get_document_content,
)

# -------------------------------
# Load env
# -------------------------------
load_dotenv()

# -------------------------------
# App
# -------------------------------
app = FastAPI(
    title="RAG Chatbot API",
    version="1.0.0",
)

# -------------------------------
# CORS
# -------------------------------
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------------------
# Globals
# -------------------------------
embedding_model = None
qdrant_cli = None
db_conn = None

# -------------------------------
# Startup
# -------------------------------
@app.on_event("startup")
def startup_event():
    global embedding_model, qdrant_cli, db_conn

    print("🔄 Starting RAG backend...")

    try:
        embedding_model = SentenceTransformer("all-MiniLM-L6-v2")
        print("✅ Embedding model loaded")
    except Exception as e:
        print("❌ Embedding load failed:", e)

    os.environ["GEMINI_API_KEY"] = settings.GEMINI_API_KEY or ""
    print("✅ Gemini API key set")

    try:
        qdrant_cli = get_qdrant_client()
        print("✅ Qdrant connected")
    except Exception as e:
        print("❌ Qdrant connection failed:", e)

    try:
        db_conn = get_db_connection()
        print("✅ PostgreSQL connected")
    except Exception as e:
        print("❌ DB connection failed:", e)


@app.on_event("shutdown")
def shutdown_event():
    if db_conn:
        db_conn.close()
        print("🛑 DB connection closed")

# -------------------------------
# Models
# -------------------------------
class ChatRequest(BaseModel):
    query: str
    selected_text: str | None = None


class ChatResponse(BaseModel):
    response: str
    sources: list[str]

# -------------------------------
# Chat API
# -------------------------------
@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):

    print("📩 Incoming query:", request.query)

    if not request.query.strip():
        raise HTTPException(status_code=400, detail="Query is empty")

    if not all([embedding_model, qdrant_cli, db_conn]):
        raise HTTPException(status_code=503, detail="RAG not initialized")

    # 1️⃣ Embed
    try:
        query_embedding = await run_in_threadpool(
            embedding_model.encode,
            request.query
        )
        query_embedding = query_embedding.tolist()
        print("✅ Embedding done")
    except Exception as e:
        print("❌ Embedding error:", e)
        raise HTTPException(500, f"Embedding failed: {e}")

    # 2️⃣ Qdrant Search
    try:
        search_result = qdrant_cli.search(
            collection_name=settings.QDRANT_COLLECTION_NAME,
            query_vector=query_embedding,
            limit=5,
            with_payload=True,
        )
        print(f"🔍 Qdrant hits: {len(search_result)}")
    except Exception as e:
        print("❌ Qdrant error:", e)
        raise HTTPException(500, f"Qdrant search failed: {e}")

    # 3️⃣ Extract IDs + sources (SAFE)
    retrieved_doc_ids = []
    retrieved_sources = []

    for hit in search_result:
        retrieved_doc_ids.append(hit.id)

        if hit.payload:
            src = hit.payload.get("source")
            if isinstance(src, str):
                retrieved_sources.append(src)

    unique_sources = sorted(set(retrieved_sources))

    # 4️⃣ DB Fetch
    try:
        if retrieved_doc_ids:
            retrieved_texts = get_document_content(
                db_conn,
                retrieved_doc_ids
            )
        else:
            retrieved_texts = []

        print(f"📄 Docs fetched: {len(retrieved_texts)}")
    except Exception as e:
        print("❌ DB error:", e)
        raise HTTPException(500, f"DB retrieval failed: {e}")

    # 5️⃣ Context
    context = (
        "\n\n".join(retrieved_texts)
        if retrieved_texts
        else "No relevant documents found."
    )

    prompt = f"""
You are a helpful assistant.

Context:
{context}

Question:
{request.query}

Answer clearly and concisely.
""".strip()

    # 6️⃣ Gemini
    try:
        response = litellm.completion(
            model="gemini/gemini-1.5-flash-latest",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.7,
        )
        llm_response = response.choices[0].message.content
        print("✅ LLM response generated")
    except Exception as e:
        print("❌ Gemini error:", e)
        raise HTTPException(500, f"LLM failed: {e}")

    return ChatResponse(
        response="TEST OK: Qdrant + DB working",
        sources=unique_sources,
    )

# -------------------------------
# Health
# -------------------------------
@app.get("/")
def root():
    return {"status": "ok"}

@app.get("/test-embed")
def test_embed():
    vec = embedding_model.encode("hello")
    return {"len": len(vec)}

