import logging
import cohere
from typing import List
from src.config.settings import settings

logger = logging.getLogger(__name__)

class CohereEmbeddingService:
    """Service for generating embeddings using Cohere API"""

    def __init__(self):
        # Validate required environment variables
        if not settings.cohere_api_key:
            logger.error("COHERE_API_KEY is not set in environment variables")
            raise ValueError("COHERE_API_KEY is required")

        logger.info(f"Initializing Cohere embedding client with API key: {'*' * (len(settings.cohere_api_key) - 4) + settings.cohere_api_key[-4:]}")

        try:
            self.client = cohere.Client(api_key=settings.cohere_api_key)
            self.model = "embed-english-v3.0"
            logger.info("Successfully initialized Cohere embedding client")
        except Exception as e:
            logger.error(f"Failed to initialize Cohere embedding client: {str(e)}")
            raise

    def generate_embeddings(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere.

        Args:
            texts: List of texts to embed
            input_type: Type of input - "search_document", "search_query", etc.

        Returns:
            List of embedding vectors
        """
        try:
            logger.info(f"Generating embeddings for {len(texts)} texts with input_type: {input_type}")
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type=input_type
            )

            logger.info(f"Generated embeddings for {len(texts)} texts with model {self.model}")
            return response.embeddings

        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")
            raise

    def generate_single_embedding(self, text: str, input_type: str = "search_document") -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to embed
            input_type: Type of input - "search_document", "search_query", etc.

        Returns:
            Embedding vector
        """
        try:
            embeddings = self.generate_embeddings([text], input_type)
            return embeddings[0]

        except Exception as e:
            logger.error(f"Error generating single embedding: {str(e)}")
            raise

    def generate_query_embedding(self, query_text: str) -> List[float]:
        """
        Generate embedding for a query text using search_query input type.

        Args:
            query_text: Query text to embed

        Returns:
            Embedding vector
        """
        return self.generate_single_embedding(query_text, input_type="search_query")

    def generate_document_embedding(self, doc_text: str) -> List[float]:
        """
        Generate embedding for a document text using search_document input type.

        Args:
            doc_text: Document text to embed

        Returns:
            Embedding vector
        """
        return self.generate_single_embedding(doc_text, input_type="search_document")


# Singleton instance
embedding_service = CohereEmbeddingService()