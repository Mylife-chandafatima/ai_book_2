import os
import asyncio
from pathlib import Path
import re
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from openai import OpenAI
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class EmbeddingGenerator:
    def __init__(self):
        # Initialize clients
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
            prefer_grpc=True
        )
        
        if os.getenv("COHERE_API_KEY"):
            self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
            self.use_cohere = True
        elif os.getenv("OPENAI_API_KEY"):
            self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
            self.use_cohere = False
        else:
            raise ValueError("Either COHERE_API_KEY or OPENAI_API_KEY must be set")
        
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "ai_book_content")
    
    def chunk_text(self, text: str, chunk_size: int = 500, overlap: int = 50) -> list[str]:
        """Split text into overlapping chunks"""
        sentences = re.split(r'[.!?]+\s+', text)
        chunks = []
        current_chunk = ""
        
        for sentence in sentences:
            if len(current_chunk) + len(sentence) < chunk_size:
                current_chunk += " " + sentence
            else:
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())
                
                # Create overlapping chunk
                words = current_chunk.split()
                overlap_words = words[-overlap:] if len(words) > overlap else words
                current_chunk = " ".join(overlap_words) + " " + sentence
        
        if current_chunk.strip():
            chunks.append(current_chunk.strip())
        
        return chunks
    
    def extract_content_from_docs(self, docs_dir: str) -> list[dict]:
        """Extract content from Docusaurus docs directory"""
        content_list = []
        docs_path = Path(docs_dir)
        
        for md_file in docs_path.rglob("*.md"):
            try:
                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                    
                    # Remove markdown headers and metadata
                    # This regex removes frontmatter
                    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)
                    
                    # Split content into chunks
                    chunks = self.chunk_text(content)
                    
                    for i, chunk in enumerate(chunks):
                        content_list.append({
                            "id": f"{md_file.name}_{i}",
                            "content": chunk,
                            "source": str(md_file),
                            "title": md_file.stem
                        })
            except Exception as e:
                logger.error(f"Error processing file {md_file}: {str(e)}")
        
        return content_list
    
    async def generate_embeddings(self, text: str) -> list[float]:
        """Generate embeddings for text"""
        try:
            if self.use_cohere:
                response = self.cohere_client.embed(
                    texts=[text],
                    model="embed-english-v3.0",
                    input_type="search_document"
                )
                return response.embeddings[0]
            else:
                response = self.openai_client.embeddings.create(
                    input=[text],
                    model="text-embedding-ada-002"
                )
                return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {str(e)}")
            raise
    
    async def populate_qdrant(self, content_list: list[dict]):
        """Populate Qdrant with content and embeddings"""
        try:
            # Determine vector size based on embedding model
            vector_size = 1024 if self.use_cohere else 1536  # Cohere: 1024, OpenAI: 1536
            
            # Create collection if it doesn't exist
            collections = self.qdrant_client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)
            
            if not collection_exists:
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=vector_size,
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            
            # Prepare points for insertion
            points = []
            for i, item in enumerate(content_list):
                embedding = await self.generate_embeddings(item["content"])
                
                point = models.PointStruct(
                    id=i,
                    vector=embedding,
                    payload={
                        "content": item["content"],
                        "source": item["source"],
                        "title": item["title"]
                    }
                )
                points.append(point)
                
                # Batch insert every 100 points to avoid memory issues
                if len(points) >= 100:
                    self.qdrant_client.upsert(
                        collection_name=self.collection_name,
                        points=points
                    )
                    logger.info(f"Upserted {len(points)} points to Qdrant")
                    points = []
            
            # Insert remaining points
            if points:
                self.qdrant_client.upsert(
                    collection_name=self.collection_name,
                    points=points
                )
                logger.info(f"Upserted final {len(points)} points to Qdrant")
            
            logger.info(f"Successfully populated Qdrant with {len(content_list)} content items")
        except Exception as e:
            logger.error(f"Error populating Qdrant: {str(e)}")
            raise
    
    async def run(self, docs_dir: str = "../docs"):
        """Run the full embedding generation and Qdrant population process"""
        logger.info("Starting embedding generation process...")
        
        # Extract content from docs
        logger.info(f"Extracting content from {docs_dir}...")
        content_list = self.extract_content_from_docs(docs_dir)
        logger.info(f"Extracted {len(content_list)} content chunks")
        
        # Populate Qdrant
        logger.info("Populating Qdrant with embeddings...")
        await self.populate_qdrant(content_list)
        
        logger.info("Embedding generation and Qdrant population completed successfully!")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Generate embeddings and populate Qdrant")
    parser.add_argument("--docs_dir", type=str, default="../docs", help="Path to docs directory")
    args = parser.parse_args()
    
    generator = EmbeddingGenerator()
    asyncio.run(generator.run(args.docs_dir))