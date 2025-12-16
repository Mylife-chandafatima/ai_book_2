import logging
from typing import List, Optional
from openai import OpenAI
from src.models.document_chunk import DocumentChunk
from src.models.response import Response, Citation
from src.config.settings import settings
from src.services.citation_service import CitationService


logger = logging.getLogger(__name__)


class GenerationService:
    """Service for generating responses using LLM and handling citations"""

    def __init__(self):
        self.openai_client = OpenAI(
            api_key=settings.openai_router_api_key,
            base_url=settings.openai_router_base_url
        )
        self.citation_service = CitationService()

    def create_context_prompt(self, chunks: List[DocumentChunk], query: str, mode: str = "book") -> str:
        """
        Create a context prompt for the LLM based on retrieved chunks.

        Args:
            chunks: List of relevant document chunks
            query: Original user query
            mode: Query mode ('book' or 'selection')

        Returns:
            Formatted context prompt string
        """
        if not chunks:
            if mode == "book":
                return f"Question: {query}\n\nContext: No relevant information found in the book.\n\nPlease respond that you cannot find relevant information in the book to answer this question."
            else:  # selection mode
                return f"Question: {query}\n\nSelected Text: \n\nPlease respond that the selected text does not contain relevant information for the question."

        # Build context from chunks
        context_parts = []
        for i, chunk in enumerate(chunks):
            context_parts.append(
                f"Document {i+1}:\n"
                f"Module: {chunk.module}\n"
                f"Chapter: {chunk.chapter}\n"
                f"Section: {chunk.section}\n"
                f"Content: {chunk.content}\n"
            )

        context = "\n".join(context_parts)

        if mode == "book":
            prompt = (
                f"Please answer the following question based ONLY on the provided context from the book. "
                f"If the context does not contain enough information to answer the question, please say so explicitly. "
                f"Do not make up information that is not in the context.\n\n"
                f"Question: {query}\n\n"
                f"Context:\n{context}\n\n"
                f"Provide your answer with proper citations to the specific modules and chapters where the information comes from."
            )
        else:  # selection mode
            # For selection mode, only use the first chunk which contains the selected text
            selected_content = chunks[0].content if chunks else ''
            prompt = (
                f"Please answer the following question based ONLY on the provided selected text. "
                f"If the selected text does not contain enough information to answer the question, please say so explicitly. "
                f"Do not make up information that is not in the selected text.\n\n"
                f"Question: {query}\n\n"
                f"Selected Text: {selected_content}\n\n"
                f"Provide your answer with proper citations to the source."
            )

        return prompt

    def update_citation_logic_for_selection_mode(self, response_text: str, chunks: List[DocumentChunk]) -> List[Citation]:
        """
        Update citation logic specifically for Selection Mode.

        Args:
            response_text: Raw response from the LLM
            chunks: List of chunks used to generate the response (should be from selected text)

        Returns:
            List of Citation objects appropriate for Selection Mode
        """
        citations = []

        for chunk in chunks:
            # For selection mode, create a special citation that indicates it's from user selection
            if chunk.metadata and chunk.metadata.get("source") == "user_selection":
                # Generate a special URL or reference for user selection
                url = "/docs/user-selection"  # Special URL for user-selected content

                citation = Citation(
                    module="User Selection",
                    chapter="Selected Text",
                    section="User Provided Context",
                    url=url
                )
                citations.append(citation)
                break  # Only add one citation for selection mode

        return citations

    def call_llm(self, prompt: str) -> str:
        """
        Call the LLM to generate a response.

        Args:
            prompt: Formatted prompt to send to the LLM

        Returns:
            Generated response text
        """
        try:
            response = self.openai_client.chat.completions.create(
                model=settings.qwen_embedding_model,  # In a real implementation, this would be the actual LLM model
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that answers questions based only on the provided context. Do not make up information that is not in the context. Always provide citations to the specific modules and chapters where you found the information."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=1000
            )

            return response.choices[0].message.content

        except Exception as e:
            logger.error(f"Error calling LLM: {str(e)}")
            return "I encountered an error while processing your request. Please try again later."

    def extract_citations(self, response_text: str, chunks: List[DocumentChunk]) -> List[Citation]:
        """
        Extract citations from the LLM response based on the source chunks.

        Args:
            response_text: Raw response from the LLM
            chunks: List of chunks used to generate the response

        Returns:
            List of Citation objects
        """
        citations = []
        processed_chunks = set()

        for chunk in chunks:
            # Create a citation for each unique chunk that was used
            if chunk.chunk_id not in processed_chunks:
                # Use the enhanced citation service to create detailed citations
                citation = self.citation_service.enhance_citation_with_detailed_reference(
                    module=chunk.module,
                    chapter=chunk.chapter,
                    section=chunk.section,
                    document_path=chunk.metadata.get("document_path", "") if chunk.metadata else ""
                )
                citations.append(citation)
                processed_chunks.add(chunk.chunk_id)

        return citations

    def format_citations_for_response(self, citations: List[Citation]) -> List[dict]:
        """
        Format citations specifically for API response to frontend.

        Args:
            citations: List of Citation objects

        Returns:
            List of citations formatted for frontend display
        """
        formatted_citations = []
        for citation in citations:
            formatted_citation = self.citation_service.format_citation_for_frontend(citation)
            formatted_citations.append(formatted_citation)

        return formatted_citations

    def generate_response_for_book_mode(self, query: str, chunks: List[DocumentChunk]) -> Response:
        """
        Generate response for Book Mode.

        Args:
            query: User's question
            chunks: Retrieved document chunks

        Returns:
            Response object with answer and citations
        """
        try:
            # Create context prompt
            prompt = self.create_context_prompt(chunks, query, mode="book")

            # Call LLM
            raw_response = self.call_llm(prompt)

            # Check if the response indicates no relevant information was found
            no_info_indicators = [
                "no relevant information",
                "cannot find",
                "not enough information",
                "not in the context",
                "no information found"
            ]

            was_refused = any(indicator.lower() in raw_response.lower() for indicator in no_info_indicators)

            if was_refused:
                return Response(
                    query_id=None,  # Will be set by the calling function
                    answer=None,
                    citations=[],
                    confidence_score=0.0,
                    was_refused=True,
                    refusal_reason="No relevant content found in the book corpus"
                )

            # Extract citations from the response
            citations = self.extract_citations(raw_response, chunks)

            # Calculate a basic confidence score based on the number of chunks used
            confidence_score = min(len(chunks) * 0.2, 1.0) if chunks else 0.0

            return Response(
                query_id=None,  # Will be set by the calling function
                answer=raw_response,
                citations=citations,
                confidence_score=confidence_score,
                was_refused=False,
                refusal_reason=None
            )

        except Exception as e:
            logger.error(f"Error generating response for Book Mode: {str(e)}")
            return Response(
                query_id=None,
                answer=None,
                citations=[],
                confidence_score=0.0,
                was_refused=True,
                refusal_reason=f"Error processing request: {str(e)}"
            )

    def generate_response_for_selection_mode(self, query: str, chunks: List[DocumentChunk]) -> Response:
        """
        Generate response for Selection Mode.

        Args:
            query: User's question
            chunks: Retrieved document chunks (should be based on selected text)

        Returns:
            Response object with answer and citations
        """
        try:
            # Create context prompt for selection mode
            prompt = self.create_context_prompt(chunks, query, mode="selection")

            # Call LLM
            raw_response = self.call_llm(prompt)

            # Check if the response indicates no relevant information was found
            no_info_indicators = [
                "no relevant information",
                "cannot find",
                "not enough information",
                "not in the context",
                "no information found",
                "selected text does not contain"
            ]

            was_refused = any(indicator.lower() in raw_response.lower() for indicator in no_info_indicators)

            if was_refused:
                return Response(
                    query_id=None,  # Will be set by the calling function
                    answer=None,
                    citations=[],
                    confidence_score=0.0,
                    was_refused=True,
                    refusal_reason="Selected text does not contain relevant information for the question"
                )

            # Extract citations from the response
            citations = self.extract_citations(raw_response, chunks)

            # Calculate a basic confidence score
            confidence_score = 0.8 if citations else 0.5  # Higher confidence for selection mode when citations exist

            return Response(
                query_id=None,  # Will be set by the calling function
                answer=raw_response,
                citations=citations,
                confidence_score=confidence_score,
                was_refused=False,
                refusal_reason=None
            )

        except Exception as e:
            logger.error(f"Error generating response for Selection Mode: {str(e)}")
            return Response(
                query_id=None,
                answer=None,
                citations=[],
                confidence_score=0.0,
                was_refused=True,
                refusal_reason=f"Error processing request: {str(e)}"
            )

    def calculate_confidence(self, chunks: List[DocumentChunk], similarity_scores: List[float]) -> float:
        """
        Calculate confidence score based on chunk relevance and similarity.

        Args:
            chunks: List of retrieved chunks
            similarity_scores: List of similarity scores for the chunks

        Returns:
            Confidence score between 0.0 and 1.0
        """
        if not similarity_scores:
            return 0.0

        # Calculate average similarity score
        avg_similarity = sum(similarity_scores) / len(similarity_scores)

        # Adjust based on number of chunks (more relevant chunks = higher confidence)
        chunk_factor = min(len(chunks) * 0.1, 0.3)  # Cap at 0.3 to not overwhelm similarity

        # Combine to get final confidence
        confidence = min(avg_similarity + chunk_factor, 1.0)

        return confidence