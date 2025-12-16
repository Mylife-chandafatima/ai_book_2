from fastapi import Depends, HTTPException, status, Request
from sqlalchemy.orm import Session
from typing import Generator
from src.config.database import SessionLocal
from datetime import datetime, timedelta
from collections import defaultdict
import time


# Simple in-memory rate limiter (for production, use Redis or similar)
request_counts = defaultdict(list)


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


def rate_limit(max_requests: int = 100, window_seconds: int = 60):
    """
    Rate limiting dependency.

    Args:
        max_requests: Maximum number of requests allowed
        window_seconds: Time window in seconds
    """
    def rate_limit_dependency(request: Request) -> bool:
        client_ip = request.client.host
        now = time.time()

        # Clean old requests outside the window
        request_counts[client_ip] = [
            req_time for req_time in request_counts[client_ip]
            if now - req_time < window_seconds
        ]

        # Check if limit exceeded
        if len(request_counts[client_ip]) >= max_requests:
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail=f"Rate limit exceeded. Maximum {max_requests} requests per {window_seconds} seconds."
            )

        # Add current request
        request_counts[client_ip].append(now)

        return True

    return Depends(rate_limit_dependency)


def rate_limit_chat(max_requests: int = 100, window_seconds: int = 60):
    """
    Rate limiting for chat endpoints.
    """
    return rate_limit(max_requests, window_seconds)


def rate_limit_ingestion(max_requests: int = 50, window_seconds: int = 60):
    """
    Rate limiting for ingestion endpoints (typically lower limits).
    """
    return rate_limit(max_requests, window_seconds)