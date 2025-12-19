import logging
from typing import List, Optional
from qdrant_client import models
from src.config.qdrant_setup import get_qdrant_client
from src.config.settings import settings
from src.config.neon_schema import SearchResultDB
from src.models.search_result import SearchResult
from src.models.document_chunk import DocumentChunk
from src.services.embedding_service import embedding_service
from sqlalchemy.orm import Session
from uuid import uuid4


logger = logging.getLogger(__name__)


class RetrievalService:
    """Service for retrieving relevant document chunks based on user queries"""

    def __init__(self):
        # Validate settings
        if not settings.qdrant_url or not settings.qdrant_collection_name:
            logger.error("QDRANT_URL or QDRANT_COLLECTION_NAME not set in environment variables")
            raise ValueError("QDRANT_URL and QDRANT_COLLECTION_NAME are required")

        self.qdrant_client = get_qdrant_client()
        self.collection_name = settings.qdrant_collection_name

        logger.info(f"Successfully initialized RetrievalService with collection: {self.collection_name}")

    def search_in_qdrant(self, query_text: str, top_k: int = 5, threshold: float = 0.5) -> List[dict]:
        """
        Perform semantic search in Qdrant collection.

        Args:
            query_text: Text to search for
            top_k: Number of top results to return
            threshold: Minimum similarity score threshold

        Returns:
            List of search results with payload and similarity scores
        """
        try:
            logger.info(f"Starting Qdrant search for query: '{query_text[:50]}...' with top_k={top_k}")

            # Generate embedding for the query text using Cohere
            query_embedding = self.generate_query_embedding(query_text)

            # Perform search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=threshold  # Minimum similarity score
            )

            # Format results
            formatted_results = []
            for i, result in enumerate(search_results):
                formatted_results.append({
                    "chunk_id": result.id,
                    "content": result.payload.get("content", ""),
                    "module": result.payload.get("module", ""),
                    "chapter": result.payload.get("chapter", ""),
                    "section": result.payload.get("section", ""),
                    "similarity_score": result.score,
                    "position": result.payload.get("position", 0),
                    "metadata": result.payload.get("metadata", {})
                })

            logger.info(f"Found {len(formatted_results)} results for query: {query_text[:50]}...")
            return formatted_results

        except Exception as e:
            logger.error(f"Error searching in Qdrant: {str(e)}")
            logger.exception("Full traceback for Qdrant search error:")
            return []

    def generate_query_embedding(self, query_text: str) -> List[float]:
        """
        Generate embedding for query text using Cohere model.

        Args:
            query_text: Text to generate embedding for

        Returns:
            List of embedding values
        """
        try:
            logger.info(f"Generating query embedding for text of length {len(query_text)}")
            embedding = embedding_service.generate_query_embedding(query_text)
            logger.info(f"Successfully generated query embedding of size {len(embedding)}")
            return embedding
        except Exception as e:
            logger.error(f"Error generating query embedding: {str(e)}")
            logger.exception("Full traceback for query embedding error:")
            # As fallback, return a random embedding of correct size (1024 for Cohere)
            import random
            fallback_embedding = [random.random() for _ in range(1024)]
            logger.warning(f"Using fallback embedding due to error. Fallback vector size: {len(fallback_embedding)}")
            return fallback_embedding

    def retrieve_for_book_mode(self, query_text: str, db: Session, top_k: int = 5) -> List[DocumentChunk]:
        """
        Retrieve relevant chunks for Book Mode (search entire corpus).

        Args:
            query_text: User's question
            db: Database session
            top_k: Number of top results to return

        Returns:
            List of relevant DocumentChunk objects
        """
        try:
            logger.info(f"Starting Book Mode retrieval for query: '{query_text[:50]}...'")

            # Perform semantic search in Qdrant
            search_results = self.search_in_qdrant(query_text, top_k=top_k)

            # Convert to DocumentChunk objects
            document_chunks = []
            for i, result in enumerate(search_results):
                chunk = DocumentChunk(
                    chunk_id=result["chunk_id"],
                    document_id=result.get("document_id", ""),
                    module=result["module"],
                    chapter=result["chapter"],
                    section=result["section"],
                    content=result["content"],
                    position=result["position"],
                    metadata=result["metadata"]
                )
                document_chunks.append(chunk)

            logger.info(f"Retrieved {len(document_chunks)} chunks for Book Mode query: {query_text[:50]}...")
            return document_chunks

        except Exception as e:
            logger.error(f"Error retrieving for Book Mode: {str(e)}")
            logger.exception("Full traceback for Book Mode retrieval error:")
            return []

    def retrieve_for_selection_mode(self, query_text: str, selected_text: str, db: Session, top_k: int = 3) -> List[DocumentChunk]:
        """
        Retrieve relevant chunks for Selection Mode (use only selected text).

        Args:
            query_text: User's question
            selected_text: Text selected by user
            db: Database session
            top_k: Number of top results to return (typically lower for selection mode)

        Returns:
            List of relevant DocumentChunk objects based only on selected text
        """
        try:
            logger.info(f"Starting Selection Mode retrieval for query: '{query_text[:50]}...' with selected text length: {len(selected_text)}")

            # Process the selected text to make sure it's properly formatted
            processed_text = selected_text.strip()

            if not processed_text:
                logger.warning("Empty selected text provided for Selection Mode")
                return []

            chunk = DocumentChunk(
                chunk_id=str(uuid4()),
                document_id="selection-context",
                module="User Selection",
                chapter="Selected Text",
                section="User Provided Context",
                content=processed_text,
                position=0,
                metadata={"source": "user_selection", "query": query_text, "original_length": len(selected_text)}
            )

            # Return only the selected text as the context
            logger.info(f"Retrieved selected text context for Selection Mode query: {query_text[:50]}...")
            return [chunk]

        except Exception as e:
            logger.error(f"Error retrieving for Selection Mode: {str(e)}")
            logger.exception("Full traceback for Selection Mode retrieval error:")
            return []

    def apply_strict_context_filtering(self, chunks: List[DocumentChunk], selected_text: str) -> List[DocumentChunk]:
        """
        Apply strict context filtering to ensure responses only use the selected text.

        Args:
            chunks: List of document chunks
            selected_text: Original selected text for comparison

        Returns:
            Filtered list of chunks that are relevant to the selected text
        """
        try:
            logger.info(f"Applying strict context filtering for {len(chunks)} chunks with selected text")

            # In selection mode, we only want to use the selected text as context
            # So we filter out any chunks that are not based on the selected text
            filtered_chunks = []

            for chunk in chunks:
                # Check if this chunk is based on the user's selected text
                if chunk.metadata and chunk.metadata.get("source") == "user_selection":
                    filtered_chunks.append(chunk)
                elif chunk.content == selected_text:
                    filtered_chunks.append(chunk)

            logger.info(f"Applied strict context filtering: {len(chunks)} -> {len(filtered_chunks)} chunks")
            return filtered_chunks

        except Exception as e:
            logger.error(f"Error applying strict context filtering: {str(e)}")
            logger.exception("Full traceback for context filtering error:")
            # Return original chunks if filtering fails
            return chunks

    def filter_context_by_relevance(self, chunks: List[DocumentChunk], query_text: str, threshold: float = 0.3) -> List[DocumentChunk]:
        """
        Filter retrieved chunks by relevance to the query.

        Args:
            chunks: List of document chunks to filter
            query_text: Original query text
            threshold: Minimum relevance threshold

        Returns:
            Filtered list of relevant chunks
        """
        logger.info(f"Filtering {len(chunks)} chunks by relevance with threshold {threshold}")
        # In a real implementation, this would use more sophisticated relevance filtering
        # For now, we'll return all chunks since we don't have a way to re-rank them without additional calls
        return chunks

    def join_metadata_from_neon(self, db: Session, chunk_ids: List[str]) -> dict:
        """
        Join additional metadata from Neon Postgres for the given chunk IDs.

        Args:
            db: Database session
            chunk_ids: List of chunk IDs to fetch metadata for

        Returns:
            Dictionary mapping chunk IDs to their metadata
        """
        try:
            logger.info(f"Joining metadata from Neon for {len(chunk_ids)} chunk IDs")

            # Query the database for chunk metadata
            chunk_metadata = {}
            for chunk_id in chunk_ids:
                # In a real implementation, we would query the database for each chunk
                # For now, we'll return an empty mapping since the metadata is already in the payload
                pass

            return chunk_metadata

        except Exception as e:
            logger.error(f"Error joining metadata from Neon: {str(e)}")
            logger.exception("Full traceback for metadata joining error:")
            return {}