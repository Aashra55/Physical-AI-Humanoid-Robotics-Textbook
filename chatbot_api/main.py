# ===============================
# RAG Chatbot – FastAPI Backend
# ===============================

import os
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from dotenv import load_dotenv
from fastapi.middleware.cors import CORSMiddleware

from sentence_transformers import SentenceTransformer
import litellm

from core.settings import settings
from core.db import (
    get_db_connection,
    get_qdrant_client,
    get_document_content,
)

# -------------------------------
# Load environment variables
# -------------------------------
load_dotenv()

# -------------------------------
# FastAPI app
# -------------------------------
app = FastAPI(
    title="RAG Chatbot API",
    description="RAG chatbot using FastAPI, Qdrant, Sentence Transformers, and Gemini",
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
# Startup / Shutdown
# -------------------------------
@app.on_event("startup")
def startup_event():
    global embedding_model, qdrant_cli, db_conn

    print("🔄 Initializing RAG components...")

    embedding_model = SentenceTransformer("all-MiniLM-L6-v2")
    print("✅ Embedding model loaded")

    os.environ["GEMINI_API_KEY"] = settings.GEMINI_API_KEY
    print("✅ Gemini API key set")

    qdrant_cli = get_qdrant_client()
    print("✅ Qdrant client initialized")

    db_conn = get_db_connection()
    print("✅ PostgreSQL connected")


@app.on_event("shutdown")
def shutdown_event():
    if db_conn:
        db_conn.close()
        print("🛑 Database connection closed")

# -------------------------------
# Request / Response Models
# -------------------------------
class ChatRequest(BaseModel):
    query: str
    selected_text: str | None = None


class ChatResponse(BaseModel):
    response: str
    sources: list[str]

# -------------------------------
# Chat Endpoint
# -------------------------------
@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    if not all([embedding_model, qdrant_cli, db_conn]):
        raise HTTPException(
            status_code=503,
            detail="RAG components not initialized",
        )

    # 1️⃣ Embed query
    query_embedding = embedding_model.encode(request.query).tolist()

    # 2️⃣ Vector search (CORRECT QDRANT API)
    try:
        search_result = qdrant_cli.search(
            collection_name=settings.QDRANT_COLLECTION_NAME,
            query_vector=query_embedding,
            limit=5,
            with_payload=True,
        )
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Qdrant search failed: {e}",
        )

    # 3️⃣ Extract IDs + sources safely
    retrieved_doc_ids = [hit.id for hit in search_result]

    retrieved_sources = [
        hit.payload.get("source")
        for hit in search_result
        if hit.payload and "source" in hit.payload
    ]
    unique_sources = sorted(set(retrieved_sources))

    # 4️⃣ Fetch documents from DB
    try:
        retrieved_texts = get_document_content(db_conn, retrieved_doc_ids)
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Database retrieval failed: {e}",
        )

    context = (
        "\n\n".join(retrieved_texts)
        if retrieved_texts
        else "No relevant documents found."
    )

    # 5️⃣ Prompt
    prompt = f"""
Context:
{context}

Question:
{request.query}

Answer:
""".strip()

    # 6️⃣ Gemini response
    try:
        response = litellm.completion(
            model="gemini/gemini-1.5-flash-latest",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.7,
        )
        llm_response = response.choices[0].message.content
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"LLM generation failed: {e}",
        )

    return ChatResponse(
        response=llm_response,
        sources=unique_sources,
    )

# -------------------------------
# Health Check
# -------------------------------
@app.get("/")
def root():
    return {"status": "ok"}
