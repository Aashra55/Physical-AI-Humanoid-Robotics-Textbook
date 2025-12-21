# Main FastAPI application for the RAG Chatbot
import os
import json
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from dotenv import load_dotenv
import litellm # Use litellm directly
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

# Add CORS middleware to allow the frontend to connect
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins for development
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

# Global variables for models and clients, initialized on startup
embedding_model = None
qdrant_cli = None
db_conn_pool = [] # Simple connection pool for Postgres

@app.on_event("startup")
async def startup_event():
    global embedding_model, qdrant_cli, db_conn_pool
    print("Initializing RAG components on startup...")

    # Initialize SentenceTransformer model (local)
    embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
    print("Local embedding model loaded.")

    # Ensure Gemini API key is exposed for LiteLLM
    os.environ["GEMINI_API_KEY"] = settings.GEMINI_API_KEY 
    print("Gemini API Key set for LiteLLM.")

    # Initialize Qdrant client
    qdrant_cli = get_qdrant_client()
    print("Qdrant client initialized.")

    # Initialize Postgres connection pool
    db_conn_pool = [get_db_connection() for _ in range(3)] # 3 connections
    print("Postgres connection pool initialized.")

@app.on_event("shutdown")
async def shutdown_event():
    global db_conn_pool
    print("Closing database connections on shutdown...")
    for conn in db_conn_pool:
        conn.close()
    print("Database connections closed.")


class ChatRequest(BaseModel):
    query: str
    selected_text: str | None = None

class ChatResponse(BaseModel):
    response: str
    sources: list[str] | None = None

@app.post("/chat")
async def chat(request: ChatRequest):
    if not embedding_model or not qdrant_cli or not db_conn_pool:
        raise HTTPException(status_code=503, detail="RAG components not initialized. Server might be starting up.")

    # 1. Embed user query
    query_embedding = embedding_model.encode(request.query, convert_to_numpy=True).tolist()

    # 2. Retrieve relevant chunks from Qdrant
    search_result = qdrant_cli.search(
        collection_name=settings.QDRANT_COLLECTION_NAME,
        query_vector=query_embedding,
        limit=5  # Retrieve top 5 most relevant chunks
    )

    retrieved_doc_ids = [hit.id for hit in search_result]
    
    # Simple connection acquisition from pool
    conn = db_conn_pool.pop()
    try:
        retrieved_texts = get_document_content(conn, retrieved_doc_ids)
    finally:
        db_conn_pool.append(conn) # Return connection to pool

    if request.selected_text:
        context = request.selected_text
        print("Using user-selected text as context.")
    elif retrieved_texts:
        context = "\n\n".join(retrieved_texts)
        print(f"Using {len(retrieved_texts)} retrieved documents as context.")
    else:
        context = "No relevant documents found."
        print("No relevant documents found for context.")

    # 3. Construct prompt for LLM
    prompt = f"""You are a helpful assistant that answers questions about a technical textbook.
Use the following context to answer the user's question. If the answer is not in the context,
state that you don't know the answer based on the provided information.

Context:
{context}

Question: {request.query}
"""
    
    # 4. Generate response directly using LiteLLM
    try:
        response = litellm.completion(
            model="gemini/gemini-1.5-flash-latest",
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.7
        )
        llm_response = response.choices[0].message.content
    except Exception as e:
        print(f"Error generating LLM response: {e}")
        llm_response = f"Sorry, I encountered an error when trying to generate a response: {e}"

    # Extract source filenames for the response
    retrieved_sources = [hit.payload['source'] for hit in search_result if hit.payload and 'source' in hit.payload]
    unique_sources = sorted(list(set(retrieved_sources)))

    return ChatResponse(response=llm_response, sources=unique_sources)

@app.get("/")
def read_root():
    """A simple health check endpoint."""
    return {"status": "ok", "message": "RAG Chatbot API is running!"}

@app.get("/check-key")
def check_key():
    """Checks if the Gemini API key is loaded from the environment."""
    api_key = os.getenv("GEMINI_API_KEY")
    if api_key:
        return {"message": "Gemini API key is loaded.", "key_loaded": True, "partial_key": f"***{api_key[-4:]}"}
    else:
        return {"message": "Gemini API key is NOT loaded.", "key_loaded": False}