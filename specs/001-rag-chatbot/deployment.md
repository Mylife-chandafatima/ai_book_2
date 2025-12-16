# Deployment Guide: RAG Chatbot for Docusaurus Book

## Overview

This guide provides instructions for deploying the integrated RAG chatbot with your Docusaurus book using free-tier services.

## Architecture

- **Frontend**: Docusaurus static site (hosted on GitHub Pages, Vercel, or Netlify)
- **Backend**: FastAPI application (hosted on Railway, Render, or similar)
- **Vector Database**: Qdrant Cloud (Free Tier)
- **Relational Database**: Neon Serverless Postgres (Free Tier)
- **LLM Service**: OpenAI Router

## Prerequisites

- Qdrant Cloud account (Free Tier)
- Neon Serverless Postgres account (Free Tier)
- OpenAI Router access
- Qwen embedding model access
- Domain name (optional)

## Backend Deployment

### 1. Environment Configuration

Create a `.env` file with your service credentials:

```env
# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.tech:6333
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=book_chunks

# Neon Postgres Configuration
NEON_DATABASE_URL=postgresql://username:password@ep-xxxx.us-east-1.aws.neon.tech/dbname

# OpenAI Router Configuration
OPENAI_ROUTER_API_KEY=your-openai-router-key
OPENAI_ROUTER_BASE_URL=https://your-router-endpoint.com

# Qwen Embedding Model
QWEN_EMBEDDING_MODEL=qwen-embedding-v1

# Application Settings
LOG_LEVEL=INFO
MAX_CONCURRENT_REQUESTS=100
```

### 2. Platform-Specific Deployment

#### Railway

1. Create a new Railway project
2. Connect to your GitHub repository
3. Set the build command: `pip install -r requirements.txt`
4. Set the start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
5. Add your environment variables in the Variables section
6. Deploy

#### Render

1. Create a new Web Service
2. Connect to your GitHub repository
3. Set the runtime to Python
4. Set the build command: `pip install -r requirements.txt`
5. Set the start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
6. Add your environment variables
7. Deploy

### 3. Ingest Book Content

After deploying the backend, you need to ingest your book content:

```bash
# Call the ingestion endpoint
curl -X POST https://your-backend-url/api/v1/ingest
```

## Frontend Deployment

### 1. Update API Base URL

Update the `REACT_APP_API_BASE_URL` in your deployment environment to point to your deployed backend:

```env
REACT_APP_API_BASE_URL=https://your-backend-url/api/v1
```

### 2. Platform Deployment

#### GitHub Pages

1. Update `docusaurus.config.js` with your domain settings
2. Run `npm run deploy`

#### Vercel

1. Push your code to GitHub
2. Import the project in Vercel
3. Set build command to `npm run build`
4. Set output directory to `build`
5. Add environment variables

#### Netlify

1. Push your code to GitHub
2. Import the project in Netlify
3. Set build command to `npm run build`
4. Set publish directory to `build`
5. Add environment variables

## Configuration

### CORS Settings

The backend is configured to allow all origins during development. For production, update the CORS settings in `backend/src/main.py`:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://yourdomain.com"],  # Update with your domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Rate Limiting

The application includes rate limiting to prevent abuse. The default settings are:

- 100 requests per minute per IP for general endpoints
- 50 requests per minute per IP for ingestion endpoints

These can be adjusted in the dependencies.

## Monitoring

### Health Checks

The application provides a health check endpoint:
- `GET /api/v1/health` - Returns the status of all services

### Performance Metrics

The application logs performance metrics for each query:
- Response time
- Success/failure status
- Error messages
- User and session information

## Security

### Input Validation

All user inputs are validated and sanitized:
- Query length limits
- Content filtering
- SQL injection prevention
- XSS prevention

### Authentication

The current implementation doesn't require authentication for basic functionality. For production use, consider implementing:
- API key authentication
- User session management
- Rate limiting per user

## Scaling Considerations

### Free Tier Limits

- Qdrant Cloud: 100MB storage, limited operations
- Neon Postgres: 10 active connections, 512MB storage
- Embedding model usage limits

### Performance Optimization

- Enable caching for frequently asked questions
- Implement query result caching
- Optimize vector search with indexing
- Use CDN for static assets

## Troubleshooting

### Common Issues

1. **Connection Timeout**: Verify all service URLs and credentials are correct
2. **Embedding Errors**: Check Qwen model access and API keys
3. **Database Errors**: Verify Neon Postgres connection string
4. **CORS Errors**: Check frontend and backend domain configurations

### Logs

Check your platform's logs for detailed error information:
- Railway: Dashboard → Logs
- Render: Dashboard → Logs
- Vercel: Dashboard → Logs

## Maintenance

### Regular Tasks

1. Monitor database storage usage
2. Check embedding model usage quotas
3. Review performance metrics
4. Update dependencies regularly

### Backup Strategy

- Regular database backups (Neon provides automatic backups)
- Version control for code and configurations
- Document environment variables and configurations

## Cost Management

### Free Tier Usage

- Qdrant Cloud: Monitor storage and operation limits
- Neon Postgres: Monitor connection time and storage
- Embedding API: Monitor token usage

### Cost Optimization

- Implement caching to reduce API calls
- Use efficient chunking to minimize embedding requests
- Monitor and optimize query patterns