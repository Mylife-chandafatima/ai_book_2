import re
from typing import Optional
from pydantic import BaseModel, field_validator


class QueryValidationResult(BaseModel):
    """Result of query validation"""
    is_valid: bool
    error_message: Optional[str] = None
    sanitized_query: str = ""


class DocumentValidationResult(BaseModel):
    """Result of document validation"""
    is_valid: bool
    error_message: Optional[str] = None
    warnings: list[str] = []


def validate_user_query(question: str, mode: str = "book", selected_text: Optional[str] = None) -> QueryValidationResult:
    """
    Validate user query based on requirements.

    Args:
        question: The user's question
        mode: Query mode ('book' or 'selection')
        selected_text: Text selected by user in selection mode

    Returns:
        QueryValidationResult with validation status
    """
    # Check question length (10-500 characters)
    if len(question) < 10:
        return QueryValidationResult(
            is_valid=False,
            error_message="Question must be at least 10 characters long"
        )

    if len(question) > 500:
        return QueryValidationResult(
            is_valid=False,
            error_message="Question must be no more than 500 characters long"
        )

    # Validate mode
    if mode not in ["book", "selection"]:
        return QueryValidationResult(
            is_valid=False,
            error_message="Mode must be either 'book' or 'selection'"
        )

    # For selection mode, check selected text
    if mode == "selection":
        if not selected_text:
            return QueryValidationResult(
                is_valid=False,
                error_message="Selected text is required for selection mode"
            )

        if len(selected_text) < 10:
            return QueryValidationResult(
                is_valid=False,
                error_message="Selected text must be at least 10 characters long"
            )

        if len(selected_text) > 2000:
            return QueryValidationResult(
                is_valid=False,
                error_message="Selected text must be no more than 2000 characters long"
            )

    # Sanitize the query (remove potentially harmful content)
    sanitized = sanitize_text(question)

    return QueryValidationResult(
        is_valid=True,
        sanitized_query=sanitized
    )


def validate_document_chunk(content: str, module: str, chapter: str) -> DocumentValidationResult:
    """
    Validate document chunk based on requirements.

    Args:
        content: The chunk content
        module: Module identifier
        chapter: Chapter identifier

    Returns:
        DocumentValidationResult with validation status
    """
    errors = []
    warnings = []

    # Check content length (50-800 tokens/characters)
    if len(content) < 50:
        errors.append("Content must be at least 50 characters long")

    if len(content) > 800:
        errors.append("Content must be no more than 800 characters long")

    # Check module and chapter
    if not module or not module.strip():
        errors.append("Module is required")

    if not chapter or not chapter.strip():
        errors.append("Chapter is required")

    if errors:
        return DocumentValidationResult(
            is_valid=False,
            error_message="; ".join(errors),
            warnings=warnings
        )

    return DocumentValidationResult(
        is_valid=True,
        warnings=warnings
    )


def sanitize_text(text: str) -> str:
    """
    Sanitize text input to prevent injection attacks.

    Args:
        text: Input text to sanitize

    Returns:
        Sanitized text
    """
    # Remove potentially harmful characters/sequences
    sanitized = re.sub(r'<script[^>]*>.*?</script>', '', text, flags=re.IGNORECASE | re.DOTALL)
    sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)
    sanitized = re.sub(r'vbscript:', '', sanitized, flags=re.IGNORECASE)
    sanitized = re.sub(r'on\w+\s*=', '', sanitized, flags=re.IGNORECASE)

    # Remove excessive whitespace
    sanitized = ' '.join(sanitized.split())

    return sanitized


def validate_embedding_vector(embedding: list) -> bool:
    """
    Validate embedding vector format.

    Args:
        embedding: The embedding vector to validate

    Returns:
        True if valid, False otherwise
    """
    if not isinstance(embedding, list):
        return False

    if len(embedding) == 0:
        return False

    # Check that all elements are numbers
    for item in embedding:
        if not isinstance(item, (int, float)):
            return False

    return True


def validate_similarity_score(score: float) -> bool:
    """
    Validate similarity score is between 0.0 and 1.0.

    Args:
        score: Similarity score to validate

    Returns:
        True if valid, False otherwise
    """
    return 0.0 <= score <= 1.0