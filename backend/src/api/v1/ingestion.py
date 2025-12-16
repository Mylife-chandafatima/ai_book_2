from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional, List

from src.services.ingestion_service import IngestionService
from src.config.database import get_db
from src.config.settings import settings


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