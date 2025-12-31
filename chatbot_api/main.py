from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
from sentence_transformers import SentenceTransformer # Re-introduced
import litellm
from fastapi.middleware.cors import CORSMiddleware
import logging # Import logging module

# Configure basic logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

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
    "https://aashra-rag-chatbot-api.hf.space", # Your Hugging Face Space URL
    "*", # Wildcard for development
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
embedding_model = SentenceTransformer("all-MiniLM-L6-v2") # Re-introduced
VECTOR_SIZE = 384 # Re-introduced

# ---------- STARTUP: CHECK QDRANT COLLECTION ----------
@app.on_event("startup")
def check_qdrant_collection():
    """
    On startup, verify that the Qdrant collection exists and has the correct
    vector size. If not, raise an error to prevent the app from starting.
    """
    try:
        collection_info = qdrant_client.get_collection(collection_name=settings.QDRANT_COLLECTION_NAME)
        # We need the vector size for the check, but fastembed handles embedding internally.
        # So we expect Qdrant to have created a collection with the appropriate fastembed model size.
        # Default fastembed model size (from 'all-MiniLM-L6-v2') is 384.
        expected_vector_size = 384 

        # Access the specific named vector config
        named_vector_config = collection_info.config.params.vectors.get("fast-bge-small-en")

        if not named_vector_config:
            logger.error(f"Qdrant collection '{settings.QDRANT_COLLECTION_NAME}' does not have the expected named vector 'fast-bge-small-en'.")
            raise RuntimeError(
                f"Qdrant collection '{settings.QDRANT_COLLECTION_NAME}' is missing the required named vector 'fast-bge-small-en'. "
                "Please run the indexing script (`core/indexing.py`) to create it correctly."
            )
        
        if named_vector_config.size != expected_vector_size:
            logger.error(f"Qdrant collection '{settings.QDRANT_COLLECTION_NAME}' named vector 'fast-bge-small-en' has wrong size. Expected {expected_vector_size}, found {named_vector_config.size}.")
            raise RuntimeError(
                f"Qdrant collection '{settings.QDRANT_COLLECTION_NAME}' named vector 'fast-bge-small-en' has the wrong size. "
                f"Expected {expected_vector_size}, found {named_vector_config.size}. "
                "Please run the indexing script (`core/indexing.py`) to create it correctly."
            )
    except Exception as e:
        logger.error(f"Qdrant collection check failed: {e}", exc_info=True) # Log startup error
        raise RuntimeError(
            f"Qdrant collection '{settings.QDRANT_COLLECTION_NAME}' not found or connection failed. "
            f"Please run the indexing script (`core/indexing.py`) to create it. Original error: {e}"
        )
    logger.info("✅ Qdrant collection check passed.")


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
        result = qdrant_client.query(
            collection_name=settings.QDRANT_COLLECTION_NAME,
            query_text=query.question, # The client will handle embedding
            limit=query.top_k,
        )

        context = []
        for i, hit in enumerate(result):
            # The payload contains the original text content.
            # Assuming 'hit' is a ScoredPoint object which does have a .payload attribute
            if hasattr(hit, 'metadata') and hit.metadata:
                text_content = hit.metadata.get("content", "")
                if text_content:
                    context.append(text_content)
            else:
                logger.warning(f"Hit {i} object does not have an expected 'metadata' attribute or it is empty. Hit: {hit}")


        logger.info(f"Retrieved {len(context)} context chunks from Qdrant for query: '{query.question}'.")
        return [item for item in context if item] # Filter out any empty strings

    except Exception as e:
        logger.error(f"Context retrieval from Qdrant failed for query '{query.question}': {e}", exc_info=True) # Log retrieval error
        raise HTTPException(status_code=500, detail=f"Context retrieval from Qdrant failed: {e}")


# ---------- CHAT ENDPOINT (RAG) ----------
@app.post("/chat")
async def chat(query: Query):
    """
    Chatbot endpoint using Retrieval-Augmented Generation (RAG).
    1. Retrieves relevant context from the textbook (vector database).
    2. Uses an LLM to generate an answer based on the user's question and the context.
    """
    logger.info(f"Received chat request for question: '{query.question}'")
    # 1. Retrieve context
    context_chunks = await retrieve_context(query)
    if not context_chunks:
        logger.warning(f"No relevant context found for query: '{query.question}'")
        return {"response": "I'm sorry, I couldn't find any relevant information in the textbook to answer your question."}

    context_str = "\n---\n".join(context_chunks)
    logger.debug(f"Context used for LLM: \n---\n{context_str}\n---") # Use debug for potentially large context

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
        logger.info(f"Calling LLM with model: {settings.LLM_MODEL}")
        response = await litellm.acompletion(
            model=settings.LLM_MODEL, messages=messages
        )

        ai_response = response.choices[0].message.content
        logger.info(f"LLM successfully generated response for query: '{query.question}'")
        return {"response": ai_response, "context": context_chunks}

    except Exception as e:
        logger.error(f"LLM generation failed for query '{query.question}': {e}", exc_info=True) # Log LLM error with traceback
        raise HTTPException(status_code=500, detail=f"LLM generation failed: {e}")


# ---------- HEALTH CHECK ----------
@app.get("/")
def health():
    logger.info("Health check endpoint accessed.")
    return {"status": "running"}