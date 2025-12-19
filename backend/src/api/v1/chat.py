from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional
from uuid import UUID
import logging

from src.models.user_query import UserQueryCreate
from src.models.response import Response
from src.services.ingestion_service import IngestionService
from src.services.retrieval_service import RetrievalService
from src.services.generation_service import GenerationService
from src.utils.validators import validate_user_query
from src.config.database import get_db
from src.config.settings import settings


logger = logging.getLogger(__name__)

router = APIRouter(prefix=settings.api_v1_prefix, tags=["chat"])


@router.post("/chat/book", response_model=Response)
async def chat_book_mode(
    query_data: UserQueryCreate,
    db: Session = Depends(get_db)
):
    """
    Submit a question for Book Mode Q&A (searches entire book corpus)
    """
    try:
        logger.info(f"Received Book Mode query: '{query_data.question[:50]}...'")

        # Validate the query
        validation_result = validate_user_query(
            question=query_data.question,
            mode="book"
        )

        if not validation_result.is_valid:
            logger.error(f"Query validation failed: {validation_result.error_message}")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=validation_result.error_message
            )

        # Update the question with sanitized version
        query_data.question = validation_result.sanitized_query

        # Initialize services
        retrieval_service = RetrievalService()
        generation_service = GenerationService()

        logger.info("Starting retrieval for Book Mode...")

        # Retrieve relevant chunks from the book corpus
        relevant_chunks = retrieval_service.retrieve_for_book_mode(
            query_text=query_data.question,
            db=db
        )

        logger.info(f"Retrieved {len(relevant_chunks)} relevant chunks")

        if not relevant_chunks:
            logger.warning("No relevant content found, returning refusal response")
            # No relevant content found, return refusal response
            return Response(
                query_id=None,
                answer=None,
                citations=[],
                confidence_score=0.0,
                was_refused=True,
                refusal_reason="No relevant content found in the book corpus"
            )

        logger.info("Starting response generation for Book Mode...")

        # Generate response using the retrieved chunks
        response = generation_service.generate_response_for_book_mode(
            query=query_data.question,
            chunks=relevant_chunks
        )

        logger.info(f"Response generated successfully. Refused: {response.was_refused}")

        # Set the query_id in the response
        # Note: In a real implementation, you'd store the query in the database and use its ID
        response.query_id = UUID(int=0)  # Placeholder

        return response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error processing Book Mode query: {str(e)}")
        logger.exception("Full traceback for Book Mode error:")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing Book Mode query: {str(e)}"
        )


@router.post("/chat/selection", response_model=Response)
async def chat_selection_mode(
    query_data: UserQueryCreate,
    db: Session = Depends(get_db)
):
    """
    Submit a question for Selection Mode Q&A (uses only user-selected text)
    """
    try:
        logger.info(f"Received Selection Mode query: '{query_data.question[:50]}...'")

        # Validate the query
        validation_result = validate_user_query(
            question=query_data.question,
            mode="selection",
            selected_text=query_data.selected_text
        )

        if not validation_result.is_valid:
            logger.error(f"Query validation failed: {validation_result.error_message}")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=validation_result.error_message
            )

        # Update the question with sanitized version
        query_data.question = validation_result.sanitized_query

        # Initialize services
        retrieval_service = RetrievalService()
        generation_service = GenerationService()

        logger.info(f"Starting retrieval for Selection Mode with selected text length: {len(query_data.selected_text) if query_data.selected_text else 0}")

        # For selection mode, we use the selected text as context
        relevant_chunks = retrieval_service.retrieve_for_selection_mode(
            query_text=query_data.question,
            selected_text=query_data.selected_text,
            db=db
        )

        logger.info(f"Retrieved {len(relevant_chunks)} relevant chunks for Selection Mode")

        if not relevant_chunks:
            logger.warning("No relevant content found in selected text, returning refusal response")
            # No relevant content found, return refusal response
            return Response(
                query_id=None,
                answer=None,
                citations=[],
                confidence_score=0.0,
                was_refused=True,
                refusal_reason="Selected text does not contain relevant information for the question"
            )

        logger.info("Starting response generation for Selection Mode...")

        # Generate response using the selected text context
        response = generation_service.generate_response_for_selection_mode(
            query=query_data.question,
            chunks=relevant_chunks
        )

        logger.info(f"Response generated successfully. Refused: {response.was_refused}")

        # Set the query_id in the response
        # Note: In a real implementation, you'd store the query in the database and use its ID
        response.query_id = UUID(int=0)  # Placeholder

        return response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error processing Selection Mode query: {str(e)}")
        logger.exception("Full traceback for Selection Mode error:")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing Selection Mode query: {str(e)}"
        )