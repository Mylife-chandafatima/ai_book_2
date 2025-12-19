import os
import sys
from pathlib import Path

# Add the backend/src directory to the Python path
sys.path.insert(0, str(Path(__file__).parent))

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from src.config.settings import settings
from src.services.ingestion_service import IngestionService


def main():
    """Run the document ingestion process."""
    print("Starting document ingestion process...")

    # Create database engine and session
    engine = create_engine(settings.neon_database_url)
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    db = SessionLocal()

    try:
        # Initialize the ingestion service
        ingestion_service = IngestionService()

        # Load all markdown files from docs directory
        print("Loading markdown files...")
        docs_path = os.path.join(Path(__file__).parent.parent.parent, "docs")  # Path to docs from backend/src
        print(f"Looking for docs at: {docs_path}")

        if os.path.exists(docs_path):
            document_paths = ingestion_service.load_markdown_files(docs_path)
            print(f"Found {len(document_paths)} markdown files")

            if document_paths:
                # Ingest documents
                print("Starting ingestion...")
                result = ingestion_service.ingest_documents(db, document_paths=document_paths)
                print(f"Ingestion completed: {result}")
            else:
                print("No markdown files found in docs directory")
        else:
            print(f"Docs directory not found at {docs_path}")

    except Exception as e:
        print(f"Error during ingestion: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        db.close()


if __name__ == "__main__":
    main()