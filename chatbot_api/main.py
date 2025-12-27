from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
from sentence_transformers import SentenceTransformer
import litellm
from fastapi.middleware.cors import CORSMiddleware # Import CORSMiddleware

from core.settings import settings
from core.db import get_qdrant_client

# litellm will automatically pick up the API key from environment variables
# (e.g., GEMINI_API_KEY), which are loaded by the settings module.

app = FastAPI()

# Configure CORS middleware
origins = [
    "http://localhost",
    "http://localhost:8080", # Common local development port
    "https://aashra55.github.io/Physical-AI-Humanoid-Robotics-Textbook/", # Your GitHub Pages frontend URL
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
embedding_model = SentenceTransformer("all-MiniLM-L6-v2")
VECTOR_SIZE = 384

# ---------- STARTUP: CHECK QDRANT COLLECTION ----------
@app.on_event("startup")
def check_qdrant_collection():
    """
    On startup, verify that the Qdrant collection exists and has the correct
    vector size. If not, raise an error to prevent the app from starting.
    """
    try:
        collection_info = qdrant_client.get_collection(collection_name=settings.QDRANT_COLLECTION_NAME)
        if collection_info.config.params.vectors.size != VECTOR_SIZE:
            raise RuntimeError(
                f"Qdrant collection '{settings.QDRANT_COLLECTION_NAME}' has the wrong vector size. "
                f"Expected {VECTOR_SIZE}, found {collection_info.config.params.vectors.size}. "
                "Please run the indexing script (`core/indexing.py`) to create it correctly."
            )
    except Exception as e:
        # Catching broad exceptions because the client can raise different errors
        # if the collection doesn't exist.
        raise RuntimeError(
            f"Qdrant collection '{settings.QDRANT_COLLECTION_NAME}' not found or connection failed. "
            f"Please run the indexing script (`core/indexing.py`) to create it. Original error: {e}"
        )
    print("✅ Qdrant collection check passed.")


# ---------- MODELS ----------
class Query(BaseModel):
    question: str
    top_k: int = 3


# ---------- HELPER: RETRIEVE CONTEXT ----------
async def retrieve_context(query: Query) -> List[str]:
    """
    Retrieves the most relevant text chunks from Qdrant based on the query.
    """
    try:
        vec = embedding_model.encode(query.question).tolist()

        result = qdrant_client.query(
            collection_name=settings.QDRANT_COLLECTION_NAME,
            query_vector=vec,
            limit=query.top_k,
            with_payload=True,
        )

        # The payload contains the original text content.
        context = [hit.payload.get("content") or hit.payload.get("text", "") for hit in result]
        return [item for item in context if item] # Filter out any empty strings

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Context retrieval from Qdrant failed: {e}")


# ---------- CHAT ENDPOINT (RAG) ----------
@app.post("/chat")
async def chat(query: Query):
    """
    Chatbot endpoint using Retrieval-Augmented Generation (RAG).
    1. Retrieves relevant context from the textbook (vector database).
    2. Uses an LLM to generate an answer based on the user's question and the context.
    """
    # 1. Retrieve context
    context_chunks = await retrieve_context(query)
    if not context_chunks:
        return {"response": "I'm sorry, I couldn't find any relevant information in the textbook to answer your question."}

    context_str = "\n---\n".join(context_chunks)

    # 2. Generate response with LLM
    try:
        system_prompt = (
            "You are an expert AI assistant for the 'Physical AI Humanoid Robotics Textbook'. "
            "Your task is to answer the user's question based *only* on the provided context from the book. "
            "Do not use any outside knowledge. If the context does not contain the answer, "
            "state that the information is not available in the provided materials."
        )

        messages = [
            {"role": "system", "content": system_prompt},
            {
                "role": "user",
                "content": f"Here is the context from the textbook:\n\n---\n{context_str}\n---\n\nPlease answer the following question:\n{query.question}",
            },
        ]

        response = await litellm.acompletion(
            model=settings.LLM_MODEL, messages=messages
        )

        ai_response = response.choices[0].message.content
        return {"response": ai_response, "context": context_chunks}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"LLM generation failed: {e}")


# ---------- HEALTH CHECK ----------
@app.get("/")
def health():
    return {"status": "running"}