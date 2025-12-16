from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.config.settings import settings
from src.config.qdrant_setup import initialize_qdrant_collection
from src.config.neon_schema import initialize_neon_schema
import logging
import uvicorn

# Import API routers
from src.api.v1.chat import router as chat_router
from src.api.v1.ingestion import router as ingestion_router
from src.api.v1.health import router as health_router
from src.api.v1.stats import router as stats_router

# Configure logging
logging.basicConfig(level=settings.log_level)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG chatbot integrated with Docusaurus book",
    version="1.0.0",
    openapi_url=f"{settings.api_v1_prefix}/openapi.json",
    docs_url=f"{settings.api_v1_prefix}/docs",
    redoc_url=f"{settings.api_v1_prefix}/redoc"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(chat_router)
app.include_router(ingestion_router)
app.include_router(health_router)
app.include_router(stats_router)

@app.on_event("startup")
async def startup_event():
    """Initialize services on startup"""
    logger.info("Initializing Qdrant collection...")
    initialize_qdrant_collection()

    logger.info("Initializing Neon schema...")
    initialize_neon_schema()

    logger.info("Application startup complete")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown"""
    logger.info("Application shutdown")

@app.get("/")
async def root():
    """Root endpoint for basic health check"""
    return {"message": "RAG Chatbot API is running", "version": "1.0.0"}

if __name__ == "__main__":
    # Run the application with uvicorn
    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level=settings.log_level.lower()
    )