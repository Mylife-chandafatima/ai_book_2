from pydantic import BaseModel
from typing import Optional

class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    response: str

class EmbeddingRequest(BaseModel):
    text: str

class EmbeddingResponse(BaseModel):
    embeddings: list[float]