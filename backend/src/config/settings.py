from pydantic_settings import BaseSettings
from typing import Optional
import os


class Settings(BaseSettings):
    """Application configuration settings"""
    # Qdrant Configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "book_chunks")

    # Neon Postgres Configuration
    neon_database_url: str = os.getenv("NEON_DATABASE_URL", "")

    # Cohere Configuration
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")

    # Application Settings
    log_level: str = os.getenv("LOG_LEVEL", "INFO")
    max_concurrent_requests: int = int(os.getenv("MAX_CONCURRENT_REQUESTS", "100"))

    # API Configuration
    api_v1_prefix: str = "/api/v1"
    debug: bool = False

    class Config:
        env_file = ".env"


# Initialize settings
settings = Settings()