import os
import logging
from typing import List, Optional
from pathlib import Path
from qdrant_client import models
from uuid import uuid4
from datetime import datetime

from src.models.document_chunk import DocumentChunk
from src.models.document_metadata import DocumentMetadata
from src.utils.chunking import chunk_markdown_content, DocumentChunkData
from src.config.qdrant_setup import get_qdrant_client
from src.config.settings import settings
from src.config.neon_schema import DocumentChunkDB, DocumentMetadataDB
from src.services.embedding_service import embedding_service
from sqlalchemy.orm import Session
from uuid import uuid4


logger = logging.getLogger(__name__)


class IngestionService:
    """Service for ingesting and processing book content"""

    def __init__(self):
        self.qdrant_client = get_qdrant_client()
        self.collection_name = settings.qdrant_collection_name

    def load_markdown_files(self, docs_path: str = "docs/") -> List[str]:
        """
        Load all markdown files from the docs directory.

        Args:
            docs_path: Path to the docs directory

        Returns:
            List of file paths
        """
        file_paths = []
        docs_dir = Path(docs_path)

        if not docs_dir.exists():
            logger.warning(f"Docs directory does not exist: {docs_path}")
            return file_paths

        # Find all markdown files in the docs directory
        for file_path in docs_dir.rglob("*.md"):
            file_paths.append(str(file_path))

        # Also include .mdx files if they exist
        for file_path in docs_dir.rglob("*.mdx"):
            file_paths.append(str(file_path))

        logger.info(f"Found {len(file_paths)} markdown files in {docs_path}")
        return file_paths

    def process_document(self, file_path: str) -> List[DocumentChunk]:
        """
        Process a single document file into chunks.

        Args:
            file_path: Path to the document file

        Returns:
            List of DocumentChunk objects
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Generate document ID based on file path
            document_id = str(uuid4())

            # Extract module and chapter from path
            path_parts = str(file_path).replace('docs/', '').split('/')
            module = path_parts[0] if path_parts else "Unknown Module"
            chapter = path_parts[-1].replace('.md', '').replace('.mdx', '') if path_parts else "Unknown Chapter"

            # Chunk the content
            chunk_data_list = chunk_markdown_content(content, str(file_path))

            # Convert to DocumentChunk objects with proper document_id
            document_chunks = []
            for i, chunk_data in enumerate(chunk_data_list):
                document_chunk = DocumentChunk(
                    document_id=document_id,
                    module=chunk_data.module,
                    chapter=chunk_data.chapter,
                    section=chunk_data.section,
                    content=chunk_data.content,
                    position=chunk_data.position,
                    metadata=chunk_data.metadata,
                    chunk_id=uuid4()
                )
                document_chunks.append(document_chunk)

            # Create document metadata
            word_count = len(content.split())
            chunk_count = len(document_chunks)

            document_metadata = DocumentMetadata(
                document_id=document_id,
                module=module,
                chapter=chapter,
                title=chapter.replace('-', ' ').title(),
                path=str(file_path),
                word_count=word_count,
                chunk_count=chunk_count
            )

            logger.info(f"Processed {file_path}: {len(document_chunks)} chunks, {word_count} words")
            return document_chunks

        except Exception as e:
            logger.error(f"Error processing document {file_path}: {str(e)}")
            return []

    def generate_embeddings(self, text: str) -> List[float]:
        """
        Generate embeddings for text using Cohere model.

        Args:
            text: Text to generate embeddings for

        Returns:
            List of embedding values
        """
        try:
            embedding = embedding_service.generate_document_embedding(text)
            logger.info(f"Generated embedding for text of length {len(text)}")
            return embedding
        except Exception as e:
            logger.error(f"Error generating embedding for text: {str(e)}")
            raise

    def store_chunks_in_qdrant(self, chunks: List[DocumentChunk]) -> bool:
        """
        Store document chunks in Qdrant vector database.

        Args:
            chunks: List of DocumentChunk objects to store

        Returns:
            True if successful, False otherwise
        """
        try:
            points = []
            for i, chunk in enumerate(chunks):
                logger.debug(f"Processing chunk {i+1}/{len(chunks)} for embedding generation")

                # Generate embedding for the chunk content
                embedding = self.generate_embeddings(chunk.content)

                logger.debug(f"Generated embedding of size {len(embedding)} for chunk {i+1}")

                # Create a Qdrant point
                point = models.PointStruct(
                    id=chunk.chunk_id,
                    vector=embedding,
                    payload={
                        "document_id": chunk.document_id,
                        "module": chunk.module,
                        "chapter": chunk.chapter,
                        "section": chunk.section,
                        "content": chunk.content,
                        "position": chunk.position,
                        "metadata": chunk.metadata or {}
                    }
                )
                points.append(point)

                logger.debug(f"Created Qdrant point for chunk {i+1} with ID: {chunk.chunk_id}")

            logger.info(f"Uploading {len(points)} chunks to Qdrant collection: {self.collection_name}")

            # Upload points to Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Successfully stored {len(points)} chunks in Qdrant")
            return True

        except Exception as e:
            logger.error(f"Error storing chunks in Qdrant: {str(e)}")
            logger.exception("Full traceback for Qdrant storage error:")
            return False

    def store_metadata_in_neon(self, db: Session, chunks: List[DocumentChunk], metadata: DocumentMetadata) -> bool:
        """
        Store document metadata in Neon Postgres database.

        Args:
            db: Database session
            chunks: List of DocumentChunk objects
            metadata: DocumentMetadata object

        Returns:
            True if successful, False otherwise
        """
        try:
            # Store document metadata
            db_metadata = DocumentMetadataDB(
                document_id=metadata.document_id,
                module=metadata.module,
                chapter=metadata.chapter,
                title=metadata.title,
                path=metadata.path,
                word_count=metadata.word_count,
                chunk_count=metadata.chunk_count
            )
            db.add(db_metadata)

            # Store individual chunks
            for chunk in chunks:
                db_chunk = DocumentChunkDB(
                    chunk_id=str(chunk.chunk_id),
                    document_id=chunk.document_id,
                    module=chunk.module,
                    chapter=chunk.chapter,
                    section=chunk.section,
                    content=chunk.content,
                    position=chunk.position,
                    metadata=chunk.metadata
                )
                db.add(db_chunk)

            db.commit()
            logger.info(f"Successfully stored metadata for {metadata.document_id} in Neon")
            return True

        except Exception as e:
            db.rollback()
            logger.error(f"Error storing metadata in Neon: {str(e)}")
            return False

    def ingest_documents(self, db: Session, document_paths: Optional[List[str]] = None, force_reprocess: bool = False) -> dict:
        """
        Main ingestion method that processes documents and stores them in both Qdrant and Neon.

        Args:
            db: Database session
            document_paths: List of specific document paths to process (if None, processes all in docs/)
            force_reprocess: Whether to reprocess documents even if they already exist

        Returns:
            Dictionary with ingestion results
        """
        try:
            if document_paths is None:
                document_paths = self.load_markdown_files()

            total_documents = len(document_paths)
            processed_documents = 0
            failed_documents = 0

            logger.info(f"Starting ingestion of {total_documents} documents")

            for file_path in document_paths:
                try:
                    # Process the document into chunks
                    chunks = self.process_document(file_path)

                    if not chunks:
                        logger.warning(f"No chunks generated for {file_path}")
                        failed_documents += 1
                        continue

                    # Store chunks in Qdrant
                    qdrant_success = self.store_chunks_in_qdrant(chunks)

                    if not qdrant_success:
                        logger.error(f"Failed to store chunks in Qdrant for {file_path}")
                        failed_documents += 1
                        continue

                    # For metadata, we need to create a document metadata object
                    # For simplicity, we'll create one based on the first chunk
                    if chunks:
                        first_chunk = chunks[0]
                        metadata = DocumentMetadata(
                            document_id=first_chunk.document_id,
                            module=first_chunk.module,
                            chapter=first_chunk.chapter,
                            title=first_chunk.chapter.replace('-', ' ').title(),
                            path=file_path,
                            word_count=len(chunks[0].content.split()) * len(chunks),  # Rough estimate
                            chunk_count=len(chunks)
                        )

                        # Store metadata in Neon
                        neon_success = self.store_metadata_in_neon(db, chunks, metadata)

                        if neon_success:
                            processed_documents += 1
                            logger.info(f"Successfully ingested {file_path}")
                        else:
                            logger.error(f"Failed to store metadata in Neon for {file_path}")
                            failed_documents += 1
                    else:
                        failed_documents += 1

                except Exception as e:
                    logger.error(f"Error processing document {file_path}: {str(e)}")
                    failed_documents += 1

            result = {
                "job_id": str(uuid4()),
                "status": "completed",
                "total_documents": total_documents,
                "processed_documents": processed_documents,
                "failed_documents": failed_documents,
                "message": f"Ingestion completed: {processed_documents} processed, {failed_documents} failed"
            }

            logger.info(result["message"])
            return result

        except Exception as e:
            logger.error(f"Error during ingestion process: {str(e)}")
            return {
                "job_id": str(uuid4()),
                "status": "failed",
                "total_documents": len(document_paths) if document_paths else 0,
                "processed_documents": 0,
                "failed_documents": len(document_paths) if document_paths else 0,
                "message": f"Ingestion failed: {str(e)}"
            }