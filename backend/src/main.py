from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.config.settings import settings
from src.config.qdrant_setup import initialize_qdrant_collection
from src.config.neon_schema import initialize_neon_schema
import logging
import uvicorn
import os

# Import API routers
from src.api.v1.chat import router as chat_router
from src.api.v1.ingestion import router as ingestion_router
from src.api.v1.health import router as health_router
from src.api.v1.stats import router as stats_router

# Configure logging
logging.basicConfig(level=settings.log_level)
logger = logging.getLogger(__name__)

def validate_environment():
    """Validate all required environment variables are set"""
    required_vars = {
        "COHERE_API_KEY": settings.cohere_api_key,
        "QDRANT_URL": settings.qdrant_url,
        "QDRANT_API_KEY": settings.qdrant_api_key,
        "QDRANT_COLLECTION_NAME": settings.qdrant_collection_name,
        "NEON_DATABASE_URL": settings.neon_database_url
    }

    missing_vars = []
    for var_name, var_value in required_vars.items():
        if not var_value:
            missing_vars.append(var_name)

    if missing_vars:
        error_msg = f"Missing required environment variables: {', '.join(missing_vars)}"
        logger.error(error_msg)
        raise ValueError(error_msg)

    logger.info("Environment validation passed - all required variables are set")

    # Log masked values for verification (only last 4 chars)
    logger.info(f"COHERE_API_KEY: {'*' * (len(settings.cohere_api_key) - 4) + settings.cohere_api_key[-4:]}")
    logger.info(f"QDRANT_URL: {settings.qdrant_url}")
    logger.info(f"QDRANT_COLLECTION_NAME: {settings.qdrant_collection_name}")
    logger.info(f"NEON_DATABASE_URL: {'sqlite' if 'sqlite' in settings.neon_database_url.lower() else 'postgresql'}://...")

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
    logger.info("Validating environment variables...")
    validate_environment()

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