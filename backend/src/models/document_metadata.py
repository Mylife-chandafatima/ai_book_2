from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field


class DocumentMetadataBase(BaseModel):
    """Base model for document metadata with common fields"""
    document_id: str
    module: str
    chapter: str
    title: str
    path: str
    word_count: int = 0
    chunk_count: int = 0


class DocumentMetadataCreate(DocumentMetadataBase):
    """Model for creating document metadata"""
    pass


class DocumentMetadataUpdate(BaseModel):
    """Model for updating document metadata"""
    module: Optional[str] = None
    chapter: Optional[str] = None
    title: Optional[str] = None
    path: Optional[str] = None
    word_count: Optional[int] = None
    chunk_count: Optional[int] = None


class DocumentMetadata(DocumentMetadataBase):
    """Model for document metadata with all fields"""
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        from_attributes = True