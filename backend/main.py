from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import logging

# Import local modules
from models import ChatRequest, ChatResponse
from rag import RAGService

# Initialize FastAPI app
app = FastAPI(
    title="AI Book RAG Chatbot API",
    description="RAG Chatbot API for Physical AI & Robotics book",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize RAG service
rag_service = RAGService()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class HealthCheck(BaseModel):
    status: str = "OK"

@app.get("/", response_model=HealthCheck)
async def root():
    return HealthCheck()

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    try:
        logger.info(f"Received chat request: {request.message[:50]}...")
        
        # Process the chat request using RAG
        response = await rag_service.process_query(
            query=request.message,
            selected_text=request.selected_text
        )
        
        logger.info(f"Generated response: {response[:50]}...")
        
        return ChatResponse(response=response)
    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing request: {str(e)}")

@app.post("/embeddings")
async def generate_embeddings(text: str):
    try:
        # Generate embeddings for the provided text
        embedding = await rag_service.generate_embedding(text)
        return {"embeddings": embedding}
    except Exception as e:
        logger.error(f"Error generating embeddings: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error generating embeddings: {str(e)}")

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)