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
genai.configure(api_key=settings.GOOGLE_API_KEY)

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
        for hit in result:
            # First priority: Text from Qdrant payload
            if hit.payload and "content" in hit.payload:
                context.append(hit.payload["content"])
            # Fallback: Query Postgres (if payload is too big or missing)
            else:
                try:
                    conn = get_db_connection()
                    cur = conn.cursor()
                    cur.execute("SELECT content FROM documents WHERE id = %s", (str(hit.id),))
                    row = cur.fetchone()
                    if row:
                        context.append(row[0])
                    cur.close()
                    conn.close()
                except Exception as pg_err:
                    logger.error(f"Postgres fallback failed: {pg_err}")
                    
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
    context_str = "\n---\n".join(context_chunks) if context_chunks else ""

    # 3. Generate RAG Response with Gemini
    try:
        system_instruction = (
            "You are the official 'Physical AI & Humanoid Robotics Textbook' Assistant. "
            "Your primary source of truth is the 'Book Context' provided below. "
            "1. ALWAYS try to find the answer in the provided context first. "
            "2. If you find the answer, rephrase it clearly in your own words. DO NOT copy-paste directly. "
            "3. If the context is empty or doesn't mention the topic specifically, "
            "provide a high-quality explanation based on standard Robotics knowledge, "
            "but explain it as a core concept of the textbook's curriculum. "
            "4. Never start your response by saying 'The provided context doesn't mention...'. Just provide the best helpful answer."
        )

        model = genai.GenerativeModel(
            model_name="gemini-2.5-flash",
            system_instruction=system_instruction
        )

        # Better formatting for the prompt
        prompt = f"Book Context:\n{context_str}\n\nUser Question: {query.question}\n\nHelpful Response:"
        
        response = model.generate_content(prompt)
        
        return {"response": response.text, "context": context_chunks}

    except Exception as e:
        logger.error(f"LLM failed: {e}")
        return {"response": "I'm sorry, I'm having trouble processing that request right now."}

# ---------- HEALTH ----------
@app.get("/")
def health():
    return {"status": "running"}
