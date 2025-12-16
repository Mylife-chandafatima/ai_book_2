from sqlalchemy import create_engine, Column, String, Integer, DateTime, Text, Float, Boolean, JSON
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
from src.config.settings import settings
import logging

logger = logging.getLogger(__name__)

# Use the same Base as in database.py to ensure consistency
from src.config.database import Base, engine


class DocumentChunkDB(Base):
    """Database model for document chunks - stores metadata in Neon Postgres"""
    __tablename__ = "document_chunks"

    chunk_id = Column(String, primary_key=True, index=True)
    document_id = Column(String, index=True, nullable=False)
    module = Column(String, nullable=False)
    chapter = Column(String, nullable=False)
    section = Column(String, nullable=False)
    content = Column(Text, nullable=False)
    position = Column(Integer, nullable=False)
    chunk_metadata = Column("metadata", JSON)  # Use different Python attribute name but same DB column name
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)


class UserQueryDB(Base):
    """Database model for user queries"""
    __tablename__ = "user_queries"

    query_id = Column(String, primary_key=True, index=True)
    question = Column(Text, nullable=False)
    mode = Column(String, nullable=False)  # 'book' or 'selection'
    selected_text = Column(Text)
    session_id = Column(String, index=True, nullable=False)
    user_id = Column(String, index=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    processed_at = Column(DateTime)


class SearchResultDB(Base):
    """Database model for search results"""
    __tablename__ = "search_results"

    result_id = Column(String, primary_key=True, index=True)
    query_id = Column(String, index=True, nullable=False)
    chunk_id = Column(String, index=True, nullable=False)
    similarity_score = Column(Float, nullable=False)
    rank = Column(Integer, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)


class ResponseDB(Base):
    """Database model for responses"""
    __tablename__ = "responses"

    response_id = Column(String, primary_key=True, index=True)
    query_id = Column(String, index=True, nullable=False)
    answer = Column(Text)
    citations = Column(JSON)  # Store as JSON array of citation objects
    confidence_score = Column(Float, nullable=False, default=0.0)
    was_refused = Column(Boolean, nullable=False, default=False)
    refusal_reason = Column(Text)
    generated_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    processing_time_ms = Column(Integer, nullable=False, default=0)


class DocumentMetadataDB(Base):
    """Database model for document metadata"""
    __tablename__ = "document_metadata"

    document_id = Column(String, primary_key=True, index=True)
    module = Column(String, nullable=False)
    chapter = Column(String, nullable=False)
    title = Column(String, nullable=False)
    path = Column(String, nullable=False)
    word_count = Column(Integer, nullable=False, default=0)
    chunk_count = Column(Integer, nullable=False, default=0)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)


class UserSessionDB(Base):
    """Database model for user sessions"""
    __tablename__ = "user_sessions"

    session_id = Column(String, primary_key=True, index=True)
    user_id = Column(String, index=True)
    start_time = Column(DateTime, default=datetime.utcnow, nullable=False)
    last_activity = Column(DateTime, default=datetime.utcnow, nullable=False)
    query_count = Column(Integer, nullable=False, default=0)
    ended_at = Column(DateTime)


def initialize_neon_schema():
    """
    Initialize the Neon Postgres schema by creating all tables.
    """
    try:
        # Create all tables defined in the models
        Base.metadata.create_all(bind=engine)
        logger.info("Successfully created/updated Neon Postgres schema")
    except Exception as e:
        logger.error(f"Failed to initialize Neon Postgres schema: {str(e)}")
        raise


def get_db_session():
    """
    Get a database session for direct database operations.
    """
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


if __name__ == "__main__":
    # Initialize the schema when running as main
    initialize_neon_schema()