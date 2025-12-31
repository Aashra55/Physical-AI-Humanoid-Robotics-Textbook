from pydantic import field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict
import os

class Settings(BaseSettings):
    """
    Manages the application's settings and environment variables.
    """
    # Load from .env file in the current working directory
    model_config = SettingsConfigDict(env_file='.env', env_file_encoding='utf-8', extra='ignore')

    # Environment-specific settings
    GEMINI_API_KEY: str
    QDRANT_URL: str
    QDRANT_API_KEY: str
    NEON_DB_URL: str

    # Application settings
    QDRANT_COLLECTION_NAME: str = "rag_chatbot_collection"
    DOCS_PATH: str = "../website/docs"
    LLM_MODEL: str = "google/gemini-pro"

    @field_validator('*', mode='before')
    @classmethod
    def strip_whitespace(cls, v):
        if isinstance(v, str):
            return v.strip()
        return v


# Create a single instance of the settings to be used throughout the application
settings = Settings()

def check_settings():
    """
    Validates that all necessary settings are loaded.
    """
    try:
        # Accessing the attributes will trigger validation
        Settings()
        print("✅ All settings loaded and validated successfully.")
    except Exception as e:
        print(f"❌ Error loading settings: {e}")
        print("Please ensure your .env file is correctly set up in the 'chatbot_api' directory.")

if __name__ == "__main__":
    check_settings()
