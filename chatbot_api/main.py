# Main FastAPI application for the RAG Chatbot
import os
import json
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from dotenv import load_dotenv
import litellm
from sentence_transformers import SentenceTransformer
from fastapi.middleware.cors import CORSMiddleware

from core.settings import settings
from core.db import get_db_connection, get_qdrant_client, get_document_content

# Load environment variables from .env file
load_dotenv()

app = FastAPI(
    title="RAG Chatbot API",
    description="An API for the RAG chatbot powered by FastAPI, Sentence Transformers, and Gemini.",
    version="0.1.0",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global variables
embedding_model = None
qdrant_cli = None
db_conn = None

@app.on_event("startup")
def startup_event():
    global embedding_model, qdrant_cli, db_conn
    print("Initializing RAG components on startup...")
    
    embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
    print("Local embedding model loaded.")
    
    os.environ["GEMINI_API_KEY"] = settings.GEMINI_API_KEY 
    print("Gemini API Key set for LiteLLM.")
    
    qdrant_cli = get_qdrant_client()
    print("Qdrant client initialized.")
    
    db_conn = get_db_connection()
    print("Postgres connection initialized.")

@app.on_event("shutdown")
def shutdown_event():
    if db_conn:
        db_conn.close()
        print("Database connection closed.")

class ChatRequest(BaseModel):
    query: str
    selected_text: str | None = None

class ChatResponse(BaseModel):
    response: str
    sources: list[str]

@app.post("/chat", response_model=ChatResponse)
def chat(request: ChatRequest):
    if not all([embedding_model, qdrant_cli, db_conn]):
        raise HTTPException(status_code=503, detail="RAG components not initialized.")

    # 1. Embed user query
    query_embedding = embedding_model.encode(request.query).tolist()

    # 2. Retrieve relevant chunks from Qdrant
    search_result = qdrant_cli.search(
        collection_name=settings.QDRANT_COLLECTION_NAME,
        query_vector=query_embedding,
        limit=5,
        with_payload=True
    )
    
    retrieved_doc_ids = [hit.id for hit in search_result]
    retrieved_sources = [hit.payload['source'] for hit in search_result if hit.payload]
    unique_sources = sorted(list(set(retrieved_sources)))

    try:
        retrieved_texts = get_document_content(db_conn, retrieved_doc_ids)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Database retrieval failed: {e}")

    context = "\n\n".join(retrieved_texts) if retrieved_texts else "No relevant documents found."

    # 3. Construct prompt
    prompt = f"""Context: {context}\n\nQuestion: {request.query}\n\nAnswer:"""
    
    # 4. Generate response
    try:
        response = litellm.completion(
            model="gemini/gemini-1.5-flash-latest",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.7
        )
        llm_response = response.choices[0].message.content
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"LLM generation failed: {e}")

    return ChatResponse(response=llm_response, sources=unique_sources)

@app.get("/")
def read_root():
    return {"status": "ok"}