import os
import json
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from dotenv import load_dotenv
from openai import OpenAI
from sentence_transformers import SentenceTransformer

from core.settings import settings
from core.db import get_db_connection, get_qdrant_client, get_document_content

# Load environment variables from .env file
load_dotenv()

app = FastAPI(
    title="RAG Chatbot API",
    description="An API for the RAG chatbot powered by FastAPI, Sentence Transformers, and Gemini.",
    version="0.1.0",
)

# Global variables for models and clients, initialized on startup
embedding_model = None
llm_client = None
qdrant_cli = None
db_conn_pool = [] # Simple connection pool for Postgres

@app.on_event("startup")
async def startup_event():
    global embedding_model, llm_client, qdrant_cli, db_conn_pool
    print("Initializing RAG components on startup...")

    # Initialize SentenceTransformer model (local)
    embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
    print("Local embedding model loaded.")

    # Initialize LiteLLM client for Gemini (using OpenAI client interface)
    # This assumes LiteLLM is configured to route 'gemini/gemini-1.5-flash-latest' to actual Gemini
    llm_client = OpenAI(
        base_url="https://api.openai.com/v1",  # Default OpenAI base_url, litellm intercepts
        api_key=settings.GEMINI_API_KEY # LiteLLM uses this to authenticate with Gemini
    )
    # Ensure Gemini API key is exposed for LiteLLM
    os.environ["GEMINI_API_KEY"] = settings.GEMINI_API_KEY 
    print("LiteLLM (Gemini) client initialized.")

    # Initialize Qdrant client
    qdrant_cli = get_qdrant_client()
    print("Qdrant client initialized.")

    # Initialize Postgres connection pool (simple for now)
    # In a real app, use a proper pool like `asyncpg` or `sqlalchemy`
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
    if not llm_client or not embedding_model or not qdrant_cli or not db_conn_pool:
        raise HTTPException(status_code=503, detail="RAG components not initialized. Server might be starting up.")

    # 1. Embed user query
    query_embedding = embedding_model.encode(request.query, convert_to_numpy=True).tolist()

    # 2. Retrieve relevant chunks from Qdrant
    search_result = qdrant_cli.search(
        collection_name=settings.QDRANT_COLLECTION_NAME,
        query_vector=query_embedding,
        limit=5  # Retrieve top 5 most relevant chunks
    )

    retrieved_doc_ids = [hit.payload['text_id'] for hit in search_result if hit.payload and 'text_id' in hit.payload]
    
    # Simple connection acquisition from pool
    conn = db_conn_pool.pop()
    try:
        retrieved_texts = get_document_content(conn, retrieved_doc_ids)
    finally:
        db_conn_pool.append(conn) # Return connection to pool

    if request.selected_text:
        # If user selected text, use it as the primary context
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
    
    # 4. Generate response using LiteLLM (Gemini)
    try:
        response = llm_client.chat.completions.create(
            model="gemini/gemini-1.5-flash-latest", # Use the specific Gemini model
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

    return ChatResponse(response=llm_response, sources=[f"Document {idx+1}" for idx, _ in enumerate(retrieved_texts)])

@app.get("/")
def read_root():
    """
    A simple health check endpoint.
    """
    return {"status": "ok", "message": "RAG Chatbot API is running!"}

@app.get("/check-key")
def check_key():
    """
    Checks if the Gemini API key is loaded from the environment.
    This is for debugging and should be removed in production.
    """
    api_key = os.getenv("GEMINI_API_KEY")
    if api_key:
        # Return only a partial key for security
        return {"message": "Gemini API key is loaded.", "key_loaded": True, "partial_key": f"***{api_key[-4:]}"}
    else:
        return {"message": "Gemini API key is NOT loaded.", "key_loaded": False}