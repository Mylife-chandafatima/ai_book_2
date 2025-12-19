from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from pydantic_settings import BaseSettings
import os


class DatabaseSettings(BaseSettings):
    """Database configuration settings"""
    neon_database_url: str = os.getenv("NEON_DATABASE_URL", "")

    class Config:
        env_file = ".env"
        extra = "ignore"  # Ignore extra environment variables


# Initialize settings
db_settings = DatabaseSettings()

# Create database engine
# Check if using SQLite (for development) or PostgreSQL (for production)
if db_settings.neon_database_url.startswith("sqlite"):
    # SQLite configuration
    engine = create_engine(
        db_settings.neon_database_url,
        connect_args={"check_same_thread": False},  # Required for SQLite
        echo=False  # Set to True for debugging SQL queries
    )
else:
    # PostgreSQL configuration
    engine = create_engine(
        db_settings.neon_database_url,
        pool_pre_ping=True,
        pool_recycle=300,
        echo=False  # Set to True for debugging SQL queries
    )

# Create session factory
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Base class for database models
Base = declarative_base()


def get_db():
    """Dependency to get database session"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()