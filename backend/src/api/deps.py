from fastapi import Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Generator
from src.config.database import SessionLocal


def get_db() -> Generator[Session, None, None]:
    """
    Dependency to get database session.
    Yields a database session to be used in API endpoints.
    """
    db = SessionLocal()
    try:
        yield db
    except Exception:
        db.rollback()
        raise
    finally:
        db.close()


def get_settings():
    """
    Dependency to get application settings.
    """
    from src.config.settings import settings
    return settings


def validate_api_key(api_key: str = None):
    """
    Validate API key if required for endpoints.
    Currently just a placeholder that allows all requests.
    """
    # In a real implementation, you would validate the API key
    # For now, we'll just allow all requests
    pass