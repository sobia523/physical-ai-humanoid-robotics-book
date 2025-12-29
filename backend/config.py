"""
Configuration management for RAG Chatbot Agent.

This module handles loading and managing configuration settings for the OpenAI Agent,
Qdrant, and Cohere API.
"""
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class Config:
    """Configuration class to manage application settings."""

    # OpenAI Configuration
    OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
    OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
    AGENT_MODEL = os.getenv("AGENT_MODEL", "gpt-4-turbo-preview")

    # Qdrant configuration
    QDRANT_URL = os.getenv("QDRANT_URL")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")

    # Cohere configuration
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    COHERE_MODEL = os.getenv("COHERE_MODEL", "embed-multilingual-v2.0")

    # Retrieval Configuration
    SIMILARITY_TOP_K = int(os.getenv("SIMILARITY_TOP_K", "5"))
    SIMILARITY_THRESHOLD = float(os.getenv("SIMILARITY_THRESHOLD", "0.5"))

    # Validation settings (for compatibility)
    SAMPLE_RECORD_LIMIT = int(os.getenv("SAMPLE_RECORD_LIMIT", "10"))
    VALIDATION_TIMEOUT = int(os.getenv("VALIDATION_TIMEOUT", "300"))  # 5 minutes in seconds

    @classmethod
    def validate_config(cls):
        """Validate that required configuration values are present."""
        required_vars = ["OPENAI_API_KEY", "OPENROUTER_API_KEY", "QDRANT_URL", "QDRANT_API_KEY", "COHERE_API_KEY"]
        missing_vars = []

        for var in required_vars:
            value = getattr(cls, var)
            if not value:
                missing_vars.append(var)

        if missing_vars:
            raise ValueError(f"Missing required configuration variables: {', '.join(missing_vars)}")

        return True