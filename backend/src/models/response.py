from datetime import datetime
from typing import List, Optional, Dict, Any
from pydantic import BaseModel, Field
from uuid import UUID, uuid4


class Citation(BaseModel):
    """Model for citation within a response"""
    module: str
    chapter: str
    section: str
    url: str


class ResponseBase(BaseModel):
    """Base model for response with common fields"""
    query_id: UUID
    answer: Optional[str] = None
    citations: List[Citation] = []
    confidence_score: float = 0.0
    was_refused: bool = False
    refusal_reason: Optional[str] = None


class ResponseCreate(ResponseBase):
    """Model for creating a response"""
    pass


class ResponseUpdate(BaseModel):
    """Model for updating a response"""
    answer: Optional[str] = None
    citations: Optional[List[Citation]] = None
    confidence_score: Optional[float] = None
    was_refused: Optional[bool] = None
    refusal_reason: Optional[str] = None


class Response(ResponseBase):
    """Model for response with all fields including ID"""
    response_id: UUID = Field(default_factory=uuid4)
    generated_at: datetime = Field(default_factory=datetime.utcnow)
    processing_time_ms: int = 0

    class Config:
        from_attributes = True