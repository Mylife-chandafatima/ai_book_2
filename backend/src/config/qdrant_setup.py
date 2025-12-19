from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from src.config.settings import settings
import logging

logger = logging.getLogger(__name__)


def initialize_qdrant_collection():
    """
    Initialize Qdrant collection for document chunks.
    This function creates the collection with appropriate vector configuration.
    """
    try:
        # Initialize Qdrant client
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            # Set timeout and other connection parameters as needed
        )

        # Define collection name from settings
        collection_name = settings.qdrant_collection_name

        # Check if collection already exists
        try:
            client.get_collection(collection_name=collection_name)
            logger.info(f"Collection '{collection_name}' already exists")
            return client
        except:
            logger.info(f"Collection '{collection_name}' does not exist, creating...")

        # Create collection for document embeddings
        # For Cohere embed-english-v3.0, the dimension is 1024
        client.recreate_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
        )

        logger.info(f"Successfully created Qdrant collection: {collection_name}")
        return client

    except Exception as e:
        logger.error(f"Failed to initialize Qdrant collection: {str(e)}")
        raise


def get_qdrant_client():
    """
    Get Qdrant client instance with proper configuration.
    """
    try:
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        return client
    except Exception as e:
        logger.error(f"Failed to create Qdrant client: {str(e)}")
        raise


if __name__ == "__main__":
    # Initialize the collection when running as main
    initialize_qdrant_collection()