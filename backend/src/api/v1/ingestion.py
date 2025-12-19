import logging
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional, List

from src.services.ingestion_service import IngestionService
from src.config.database import get_db
from src.config.settings import settings
from src.config.qdrant_setup import get_qdrant_client


router = APIRouter(prefix=settings.api_v1_prefix, tags=["ingestion"])


@router.post("/ingest")
async def ingest_documents(
    document_paths: Optional[List[str]] = None,
    force_reprocess: bool = False,
    db: Session = Depends(get_db)
):
    """
    Ingest and process book content from the docs/ folder
    """
    try:
        ingestion_service = IngestionService()

        result = ingestion_service.ingest_documents(
            db=db,
            document_paths=document_paths,
            force_reprocess=force_reprocess
        )

        return result

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error during ingestion: {str(e)}"
        )


@router.get("/ingestion/status")
async def get_ingestion_status(db: Session = Depends(get_db)):
    """
    Get current ingestion status including Qdrant collection information
    """
    try:
        client = get_qdrant_client()
        collection_name = settings.qdrant_collection_name
        
        # Get collection info
        collection_info = client.get_collection(collection_name=collection_name)
        
        # Get count of points in collection
        count = client.count(
            collection_name=collection_name
        )
        
        return {
            "collection_name": collection_name,
            "collection_status": collection_info.status,
            "vector_count": count.count,
            "config": {
                "vector_size": collection_info.config.params.vectors.size,
                "distance": collection_info.config.params.vectors.distance
            }
        }
        
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting ingestion status: {str(e)}"
        )


@router.post("/ingest/from-path")
async def ingest_documents_from_path(
    docs_path: str = "docs/",
    force_reprocess: bool = False,
    db: Session = Depends(get_db)
):
    """
    Ingest documents from a specific path (default is docs/)
    """
    try:
        ingestion_service = IngestionService()

        # Load documents from the specified path
        document_paths = ingestion_service.load_markdown_files(docs_path)
        
        if not document_paths:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"No markdown files found in {docs_path}"
            )

        result = ingestion_service.ingest_documents(
            db=db,
            document_paths=document_paths,
            force_reprocess=force_reprocess
        )

        return result

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error during ingestion: {str(e)}"
        )