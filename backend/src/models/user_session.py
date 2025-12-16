from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field


class UserSessionBase(BaseModel):
    """Base model for user session with common fields"""
    session_id: str
    user_id: Optional[str] = None
    query_count: int = 0


class UserSessionCreate(UserSessionBase):
    """Model for creating a user session"""
    pass


class UserSessionUpdate(BaseModel):
    """Model for updating a user session"""
    user_id: Optional[str] = None
    query_count: Optional[int] = None
    last_activity: Optional[datetime] = None


class UserSession(UserSessionBase):
    """Model for user session with all fields"""
    start_time: datetime = Field(default_factory=datetime.utcnow)
    last_activity: datetime = Field(default_factory=datetime.utcnow)
    ended_at: Optional[datetime] = None

    class Config:
        from_attributes = True