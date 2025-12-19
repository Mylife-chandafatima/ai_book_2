# Required Environment Variables for RAG Pipeline

## COHERE_API_KEY
- **Purpose**: API key for accessing Cohere's embedding and LLM services
- **Value**: Your personal Cohere API key from https://dashboard.cohere.ai/api-keys
- **Usage**: Used by both embedding_service.py and generation_service.py for all Cohere API calls

## QDRANT_URL
- **Purpose**: URL endpoint for your Qdrant vector database instance
- **Value**: Your Qdrant cluster URL (e.g., https://your-cluster.qdrant.tech:6333)
- **Usage**: Used by qdrant_setup.py and retrieval_service.py to connect to vector database

## QDRANT_API_KEY
- **Purpose**: Authentication key for your Qdrant instance
- **Value**: Your Qdrant API key from your Qdrant Cloud dashboard
- **Usage**: Used for authenticating all Qdrant operations

## QDRANT_COLLECTION_NAME
- **Purpose**: Name of the collection in Qdrant where document embeddings are stored
- **Value**: String identifier (default: "book_chunks")
- **Usage**: Used to specify which collection to read from/write to in Qdrant

## NEON_DATABASE_URL
- **Purpose**: Connection string for Neon Postgres database
- **Value**: Postgres connection string (e.g., postgresql://username:password@ep-xxxx.us-east-1.aws.neon.tech/dbname)
- **Usage**: Used by database.py and all services that interact with metadata storage

## LOG_LEVEL
- **Purpose**: Controls the verbosity of application logging
- **Value**: DEBUG, INFO, WARNING, ERROR, or CRITICAL (default: INFO)
- **Usage**: Controls logging level across all services

## MAX_CONCURRENT_REQUESTS
- **Purpose**: Limits the number of simultaneous requests the API can handle
- **Value**: Integer (default: 100)
- **Usage**: Controls request concurrency limits in the FastAPI application

## CORS_ORIGINS
- **Purpose**: Specifies which origins are allowed to make cross-origin requests
- **Value**: JSON array of allowed origins (e.g., ["http://localhost:3000", "https://your-domain.com"])
- **Usage**: Configures CORS middleware in FastAPI application

# How Each Variable is Used in the Code:

1. **COHERE_API_KEY**:
   - In settings.py: `cohere_api_key = os.getenv("COHERE_API_KEY", "")`
   - In embedding_service.py: `cohere.Client(api_key=settings.cohere_api_key)`
   - In generation_service.py: `cohere.Client(api_key=settings.cohere_api_key)`

2. **QDRANT Configuration**:
   - In settings.py: `qdrant_url`, `qdrant_api_key`, `qdrant_collection_name`
   - In qdrant_setup.py: Used to initialize QDRant client and collection
   - In retrieval_service.py and ingestion_service.py: Used for database operations

3. **NEON_DATABASE_URL**:
   - In database.py: `engine = create_engine(db_settings.neon_database_url)`
   - In all services: Used for database session management

4. **Application Settings**:
   - LOG_LEVEL: Controls logging level in main.py and all services
   - MAX_CONCURRENT_REQUESTS: Used for request limiting
   - CORS_ORIGINS: Configured in main.py middleware

# Where to Obtain Each Key:

1. **COHERE_API_KEY**: Sign up at https://cohere.ai and go to Dashboard > API Keys
2. **QDRANT Credentials**: Create account at https://qdrant.tech and get from dashboard
3. **NEON_DATABASE_URL**: Create project at https://neon.tech and get connection string from project settings