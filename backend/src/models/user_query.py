from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field
from uuid import UUID, uuid4


class UserQueryBase(BaseModel):
    """Base model for user query with common fields"""
    question: str
    mode: str  # 'book' or 'selection'
    session_id: str
    user_id: Optional[str] = None


class UserQueryCreate(UserQueryBase):
    """Model for creating a user query"""
    selected_text: Optional[str] = None


class UserQueryUpdate(BaseModel):
    """Model for updating a user query"""
    question: Optional[str] = None
    selected_text: Optional[str] = None
    session_id: Optional[str] = None
    user_id: Optional[str] = None


class UserQuery(UserQueryBase):
    """Model for user query with all fields including ID"""
    query_id: UUID = Field(default_factory=uuid4)
    selected_text: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    processed_at: Optional[datetime] = None

    class Config:
        from_attributes = True