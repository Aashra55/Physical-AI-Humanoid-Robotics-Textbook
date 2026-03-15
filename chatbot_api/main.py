from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Union
import logging
import google.generativeai as genai
from sentence_transformers import SentenceTransformer

from core.settings import settings
from core.db import get_qdrant_client, get_db_connection

# Configure basic logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Configure Gemini API
genai.configure(api_key=settings.GEMINI_API_KEY)

app = FastAPI()

# Configure CORS middleware
origins = [
    "http://localhost",
    "http://localhost:3000",
    "http://localhost:8000",
    "https://aashra55.github.io",
    "*",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---------- CLIENTS AND MODELS ----------
qdrant_client = get_qdrant_client()
logger.info("Loading SentenceTransformer model (all-MiniLM-L6-v2)...")
embedding_model = SentenceTransformer("all-MiniLM-L6-v2")

# ---------- STARTUP ----------
@app.on_event("startup")
def startup_event():
    logger.info("Checking Qdrant collection...")
    try:
        collection_name = settings.QDRANT_COLLECTION_NAME
        if not qdrant_client.collection_exists(collection_name=collection_name):
            logger.warning(f"Collection {collection_name} not found. Please run indexing.")
        else:
            logger.info(f"Collection {collection_name} verified.")
    except Exception as e:
        logger.error(f"Startup check failed: {e}")

# ---------- MODELS ----------
class Query(BaseModel):
    question: str
    top_k: int = 3

# ---------- HELPERS ----------
def is_greeting(text: str) -> bool:
    greetings = ["hi", "hello", "hey", "assalam", "salaam", "greetings", "good morning", "good afternoon", "how are you"]
    text = text.lower().strip()
    return any(text.startswith(g) or text == g for g in greetings)

async def retrieve_context(question: str, top_k: int) -> List[str]:
    try:
        vec = embedding_model.encode(question).tolist()
        result = qdrant_client.query_points(
            collection_name=settings.QDRANT_COLLECTION_NAME,
            query=vec,
            limit=top_k
        ).points
        
        context = []
        for hit in result:
            if hit.payload and "content" in hit.payload:
                context.append(hit.payload["content"])
        return context
    except Exception as e:
        logger.error(f"Retrieval failed: {e}")
        return []

# ---------- CHAT ENDPOINT ----------
@app.post("/chat")
async def chat(query: Query):
    logger.info(f"Received request: {query.question}")
    
    # 1. Handle Greetings
    if is_greeting(query.question):
        return {"response": "Hello! I am your Physical AI & Humanoid Robotics assistant. How can I help you explore the textbook today?"}

    # 2. Retrieve Context
    context_chunks = await retrieve_context(query.question, query.top_k)
    
    if not context_chunks:
        return {"response": "I couldn't find specific information about that in the book, but I'm happy to help with other topics like ROS 2, Gazebo, or Isaac Sim!"}

    context_str = "\n---\n".join(context_chunks)

    # 3. Generate RAG Response with Gemini
    try:
        system_instruction = (
            "You are an expert AI assistant for the 'Physical AI Humanoid Robotics Textbook'. "
            "Answer the user's question based ONLY on the provided context. "
            "If the answer isn't in the context, say you don't know based on the book materials."
        )

        model = genai.GenerativeModel(
            model_name=settings.LLM_MODEL,
            system_instruction=system_instruction
        )

        response = model.generate_content(
            f"Context:\n{context_str}\n\nQuestion: {query.question}"
        )
        
        return {"response": response.text, "context": context_chunks}

    except Exception as e:
        logger.error(f"LLM failed: {e}")
        # Fallback to just returning the context if LLM fails
        return {"results": context_chunks, "error": str(e)}

# ---------- HEALTH ----------
@app.get("/")
def health():
    return {"status": "running"}
