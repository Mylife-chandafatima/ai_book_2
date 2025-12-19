import asyncio
import logging
from typing import List, Optional
import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from openai import OpenAI
import numpy as np

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
            prefer_grpc=True
        )
        
        # Initialize embedding provider (Cohere or OpenAI)
        if os.getenv("COHERE_API_KEY"):
            self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
            self.use_cohere = True
        elif os.getenv("OPENAI_API_KEY"):
            self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
            self.use_cohere = False
        else:
            raise ValueError("Either COHERE_API_KEY or OPENAI_API_KEY must be set")
        
        # Initialize LLM provider (prefer OpenAI, fallback to Cohere)
        if os.getenv("OPENAI_API_KEY"):
            self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
            self.use_openai = True
        elif os.getenv("COHERE_API_KEY"):
            self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
            self.use_openai = False
        else:
            raise ValueError("Either OPENAI_API_KEY or COHERE_API_KEY must be set")
        
        # Collection name for storing embeddings
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "ai_book_content")
        
        logger.info("RAG Service initialized successfully")
    
    async def generate_embedding(self, text: str) -> List[float]:
        """Generate embeddings for the given text"""
        try:
            if self.use_cohere:
                response = self.cohere_client.embed(
                    texts=[text],
                    model="embed-english-v3.0",
                    input_type="search_query"  # or "search_document" for content
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
    
    async def retrieve_relevant_content(self, query: str, top_k: int = 5) -> List[str]:
        """Retrieve relevant content from Qdrant based on the query"""
        try:
            # Generate embedding for the query
            query_embedding = await self.generate_embedding(query)
            
            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True
            )
            
            # Extract content from search results
            relevant_content = []
            for result in search_results:
                if result.payload and 'content' in result.payload:
                    relevant_content.append(result.payload['content'])
            
            return relevant_content
        except Exception as e:
            logger.error(f"Error retrieving content: {str(e)}")
            raise
    
    async def generate_response(self, query: str, context: str) -> str:
        """Generate response using LLM with the provided context"""
        try:
            if self.use_openai:
                response = self.openai_client.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {
                            "role": "system",
                            "content": "You are an AI assistant for the Physical AI & Robotics book. Answer questions based only on the provided context. If the answer is not in the context, say 'I don't have enough information from the book to answer that question.'"
                        },
                        {
                            "role": "user",
                            "content": f"Context: {context}\n\nQuestion: {query}"
                        }
                    ],
                    max_tokens=500,
                    temperature=0.3
                )
                return response.choices[0].message.content.strip()
            else:
                # Using Cohere
                response = self.cohere_client.chat(
                    message=query,
                    documents=[{"text": context}],
                    preamble="You are an AI assistant for the Physical AI & Robotics book. Answer questions based only on the provided context. If the answer is not in the context, say 'I don't have enough information from the book to answer that question.'",
                    temperature=0.3
                )
                return response.text
        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            raise
    
    async def process_query(self, query: str, selected_text: Optional[str] = None) -> str:
        """Process a query using RAG methodology"""
        try:
            # If selected text is provided, use it as context
            if selected_text and len(selected_text.strip()) > 10:
                # Generate response based on selected text
                response = await self.generate_response(query, selected_text)
                return response
            
            # Otherwise, retrieve relevant content from the vector database
            relevant_content = await self.retrieve_relevant_content(query)
            
            if not relevant_content:
                return "I couldn't find relevant content in the book to answer your question. Please try rephrasing."
            
            # Combine all relevant content
            context = "\n\n".join(relevant_content)
            
            # Generate response based on retrieved context
            response = await self.generate_response(query, context)
            
            return response
        except Exception as e:
            logger.error(f"Error processing query: {str(e)}")
            return "Sorry, I encountered an error while processing your request. Please try again."

    def ensure_collection_exists(self):
        """Ensure the Qdrant collection exists with proper configuration"""
        try:
            # Check if collection exists
            collections = self.qdrant_client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)
            
            if not collection_exists:
                # Create collection - dimensions depend on embedding model
                # Cohere embeddings are 1024-dim, OpenAI are 1536-dim
                vector_size = 1024 if self.use_cohere else 1536
                
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=vector_size,
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error ensuring collection exists: {str(e)}")
            raise