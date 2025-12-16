from datetime import datetime
from typing import Optional, Dict, Any
from pydantic import BaseModel, Field
from uuid import UUID, uuid4


class DocumentChunkBase(BaseModel):
    """Base model for document chunk with common fields"""
    document_id: str
    module: str
    chapter: str
    section: str
    content: str
    position: int
    metadata: Optional[Dict[str, Any]] = None


class DocumentChunkCreate(DocumentChunkBase):
    """Model for creating a document chunk"""
    pass


class DocumentChunkUpdate(BaseModel):
    """Model for updating a document chunk"""
    module: Optional[str] = None
    chapter: Optional[str] = None
    section: Optional[str] = None
    content: Optional[str] = None
    position: Optional[int] = None
    metadata: Optional[Dict[str, Any]] = None


class DocumentChunk(DocumentChunkBase):
    """Model for document chunk with all fields including ID"""
    chunk_id: UUID = Field(default_factory=uuid4)
    embedding: Optional[list] = None  # Will store the embedding vector
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        from_attributes = True