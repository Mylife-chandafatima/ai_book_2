from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field
from uuid import UUID


class SearchResultBase(BaseModel):
    """Base model for search result with common fields"""
    query_id: UUID
    chunk_id: UUID
    similarity_score: float
    rank: int


class SearchResultCreate(SearchResultBase):
    """Model for creating a search result"""
    pass


class SearchResultUpdate(BaseModel):
    """Model for updating a search result"""
    similarity_score: Optional[float] = None
    rank: Optional[int] = None


class SearchResult(SearchResultBase):
    """Model for search result with all fields including ID"""
    result_id: UUID = Field(default_factory=lambda: UUID(bytes=bytes(uuid4())))
    created_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        from_attributes = True