import sys
from pathlib import Path

# Add the backend/src directory to the Python path
sys.path.append(str(Path(__file__).parent))

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from src.config.settings import settings
from src.services.ingestion_service import IngestionService
from src.services.retrieval_service import RetrievalService
from src.services.generation_service import GenerationService


def test_ingestion():
    """Test the document ingestion process."""
    print("=== Testing Document Ingestion ===")
    
    # Create database engine and session
    engine = create_engine(settings.neon_database_url)
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    db = SessionLocal()
    
    try:
        # Initialize the ingestion service
        ingestion_service = IngestionService()
        
        # Load all markdown files from docs directory
        print("Loading markdown files...")
        docs_path = "../../../docs"  # Relative to backend/src directory
        if Path(docs_path).exists():
            document_paths = ingestion_service.load_markdown_files(docs_path)
            print(f"Found {len(document_paths)} markdown files")
            
            # Test ingestion with first file only for validation
            if document_paths:
                test_paths = document_paths[:2]  # Test with first 2 files
                print(f"Processing files: {test_paths}")
                
                result = ingestion_service.ingest_documents(db, document_paths=test_paths)
                print(f"Ingestion result: {result}")
            else:
                print("No documents found to process")
        else:
            print(f"Docs directory not found at {docs_path}")
            
    except Exception as e:
        print(f"Error during ingestion test: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        db.close()


def test_retrieval():
    """Test the retrieval functionality."""
    print("\n=== Testing Document Retrieval ===")
    
    # Create database engine and session
    engine = create_engine(settings.neon_database_url)
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    db = SessionLocal()
    
    try:
        retrieval_service = RetrievalService()
        
        # Test query
        query = "What is ROS2?"
        print(f"Testing retrieval for query: '{query}'")
        
        # Test book mode retrieval
        book_chunks = retrieval_service.retrieve_for_book_mode(query, db, top_k=3)
        print(f"Book mode retrieval found {len(book_chunks)} chunks")
        
        for i, chunk in enumerate(book_chunks):
            print(f"Chunk {i+1}: Module={chunk.module}, Chapter={chunk.chapter}, Content preview: {chunk.content[:100]}...")
        
        # Test selection mode retrieval
        selected_text = "ROS2 is a robotics framework."
        selection_chunks = retrieval_service.retrieve_for_selection_mode(query, selected_text, db)
        print(f"Selection mode retrieval found {len(selection_chunks)} chunks")
        
        for i, chunk in enumerate(selection_chunks):
            print(f"Selection Chunk {i+1}: Content: {chunk.content}")
            
    except Exception as e:
        print(f"Error during retrieval test: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        db.close()


def test_generation():
    """Test the generation functionality."""
    print("\n=== Testing Response Generation ===")
    
    try:
        generation_service = GenerationService()
        
        # Test prompt
        query = "What is ROS2?"
        sample_chunks = [
            {
                "module": "ROS2 Basics",
                "chapter": "Introduction",
                "section": "Overview",
                "content": "ROS2 is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.",
                "position": 0,
                "metadata": {}
            }
        ]
        
        # For testing, we need to create DocumentChunk instances
        from src.models.document_chunk import DocumentChunk
        from uuid import uuid4
        
        document_chunks = []
        for chunk_data in sample_chunks:
            chunk = DocumentChunk(
                chunk_id=str(uuid4()),
                document_id=str(uuid4()),
                module=chunk_data["module"],
                chapter=chunk_data["chapter"],
                section=chunk_data["section"],
                content=chunk_data["content"],
                position=chunk_data["position"],
                metadata=chunk_data["metadata"]
            )
            document_chunks.append(chunk)
        
        print(f"Testing generation for query: '{query}' with {len(document_chunks)} chunks")
        
        # Test book mode generation
        book_response = generation_service.generate_response_for_book_mode(query, document_chunks)
        print(f"Book mode response was_refused: {book_response.was_refused}")
        if book_response.answer:
            print(f"Book mode response preview: {book_response.answer[:200]}...")
        
        # Test selection mode generation
        selection_response = generation_service.generate_response_for_selection_mode(query, document_chunks)
        print(f"Selection mode response was_refused: {selection_response.was_refused}")
        if selection_response.answer:
            print(f"Selection mode response preview: {selection_response.answer[:200]}...")
            
    except Exception as e:
        print(f"Error during generation test: {str(e)}")
        import traceback
        traceback.print_exc()


def main():
    """Run all tests."""
    print("Starting RAG Pipeline Tests")
    
    test_ingestion()
    test_retrieval()
    test_generation()
    
    print("\nRAG Pipeline Tests Completed")


if __name__ == "__main__":
    main()