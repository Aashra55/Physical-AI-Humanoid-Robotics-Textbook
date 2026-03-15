from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Union
import logging
from google import genai
from google.genai import types
from sentence_transformers import SentenceTransformer

from core.settings import settings
from core.db import get_qdrant_client, get_db_connection

# Configure basic logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Configure Gemini API Client
client_genai = genai.Client(api_key=settings.GOOGLE_API_KEY)

app = FastAPI()

# Super permissive CORS for local testing
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---------- CLIENTS AND MODELS ----------
qdrant_client = get_qdrant_client()
logger.info("Loading SentenceTransformer model (all-MiniLM-L6-v2)...")
embedding_model = SentenceTransformer("all-MiniLM-L6-v2")

# ---------- MODELS ----------
class Query(BaseModel):
    question: str
    top_k: int = 5

# ---------- HELPERS ----------
def is_greeting(text: str) -> bool:
    greetings = ["hi", "hello", "hey", "assalam", "salaam", "greetings", "good morning", "good afternoon", "how are you"]
    text = text.lower().strip().replace("?", "").replace("!", "")
    return any(text == g or text.startswith(g + " ") for g in greetings)

async def retrieve_context(question: str, top_k: int) -> List[str]:
    try:
        vec = embedding_model.encode(question).tolist()
        result = qdrant_client.query_points(
            collection_name=settings.QDRANT_COLLECTION_NAME,
            query=vec,
            limit=top_k
        ).points
        
        context = []
        seen = set()
        for hit in result:
            if hit.payload and "content" in hit.payload:
                content = hit.payload["content"].strip()
                # Simple de-duplication
                if content[:50] not in seen:
                    context.append(content)
                    seen.add(content[:50])
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
    
    # Even if context is thin, let the LLM try to answer if it knows it's about the book
    context_str = "\n---\n".join(context_chunks) if context_chunks else "No specific context found."

    # 3. Generate RAG Response with Gemini
    try:
        system_instruction = (
            "You are an expert AI tutor for the 'Physical AI & Humanoid Robotics' textbook. "
            "Your goal is to provide helpful, clear, and educational answers. "
            "Use the provided context from the book to answer. "
            "CRITICAL: Do not just copy and paste the context directly. Instead, rephrase and compose the information in your own words to explain it clearly and naturally to the user. "
            "If the context is missing or thin, use your general knowledge of Robotics and Physical AI to supplement the answer, "
            "but always keep the tone consistent with a textbook assistant. "
            "If you are absolutely sure the topic is not related to the book at all, let the user know."
        )

        # Use gemini-1.5-flash for better reliability
        response = client_genai.models.generate_content(
            model="gemini-1.5-flash", 
            contents=f"Context from textbook:\n{context_str}\n\nUser Question: {query.question}",
            config=types.GenerateContentConfig(
                system_instruction=system_instruction,
                temperature=0.7
            )
        )
        
        if not response.text:
            raise ValueError("Empty response from Gemini")

        return {"response": response.text, "context": context_chunks}

    except Exception as e:
        logger.error(f"LLM failed: {e}")
        # If we have context but LLM fails, at least show the context nicely
        if context_chunks:
            return {
                "response": "I found some relevant sections in the book for you, though I had trouble generating a summary:\n\n" + "\n\n".join(context_chunks),
                "context": context_chunks
            }
        return {"response": "I'm sorry, I'm having trouble processing that request right now."}

# ---------- HEALTH ----------
@app.get("/")
def health():
    return {"status": "running"}
