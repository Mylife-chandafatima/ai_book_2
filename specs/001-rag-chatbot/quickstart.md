# Quickstart Guide: RAG Chatbot for Docusaurus Book

## Prerequisites

- Python 3.11+
- Node.js 18+ (for Docusaurus development)
- Qdrant Cloud account (Free Tier)
- Neon Serverless Postgres account
- OpenAI Router access
- Qwen embedding model access

## Environment Setup

### Backend Dependencies
```bash
cd backend
pip install fastapi uvicorn qdrant-client sqlalchemy psycopg2-binary python-dotenv openai tiktoken
```

### Environment Variables
Create `.env` file in backend root:
```env
QDRANT_URL=https://your-cluster.qdrant.tech:6333
QDRANT_API_KEY=your-qdrant-api-key
NEON_DATABASE_URL=postgresql://username:password@ep-xxxx.us-east-1.aws.neon.tech/dbname
OPENAI_ROUTER_API_KEY=your-openai-router-key
OPENAI_ROUTER_BASE_URL=https://your-router-endpoint.com
QWEN_EMBEDDING_MODEL=qwen-embedding-v1
```

## Local Development Setup

### 1. Initialize the Backend
```bash
# Navigate to backend directory
cd backend

# Install dependencies
pip install -r requirements.txt

# Set up environment variables (see above)

# Run the application
uvicorn src.main:app --reload --port 8000
```

### 2. Configure Qdrant Collection
```python
# Initialize Qdrant collection for document chunks
from qdrant_client import QdrantClient
from qdrant_client.http import models

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# Create collection for document embeddings
client.recreate_collection(
    collection_name="book_chunks",
    vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
)
```

### 3. Initialize Database Tables
```bash
# Run database migrations (or create tables directly)
python -m src.database.init
```

### 4. Ingest Book Content
```bash
# Ingest all content from docs/ folder
curl -X POST http://localhost:8000/api/v1/ingest \
  -H "Content-Type: application/json" \
  -d '{"force_reprocess": false}'
```

## Frontend Integration

### 1. Install Docusaurus Plugin
```bash
cd /path/to/docusaurus/book
npm install @docusaurus/core
```

### 2. Add Chatbot Component
In your Docusaurus layout, include the chatbot widget:
```jsx
// In src/components/ChatbotWidget/index.js
import React, { useState } from 'react';

const ChatbotWidget = () => {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [mode, setMode] = useState('book'); // 'book' or 'selection'
  const [selectedText, setSelectedText] = useState('');

  const handleSubmit = async (e) => {
    e.preventDefault();

    // Call backend API based on mode
    const endpoint = mode === 'book'
      ? '/api/v1/chat/book'
      : '/api/v1/chat/selection';

    const response = await fetch(endpoint, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        question: input,
        selected_text: mode === 'selection' ? selectedText : undefined,
        session_id: 'session-123'
      })
    });

    const data = await response.json();
    setMessages([...messages, { role: 'user', content: input },
                           { role: 'assistant', content: data.answer, citations: data.citations }]);
    setInput('');
  };

  return (
    <div className="chatbot-widget">
      {/* Chat interface implementation */}
    </div>
  );
};

export default ChatbotWidget;
```

### 3. Register the Component
In your Docusaurus configuration:
```js
// docusaurus.config.js
module.exports = {
  // ... other config
  themes: [
    // ... other themes
  ],
  plugins: [
    // ... other plugins
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'chatbot',
        path: 'src/components/ChatbotWidget',
        routeBasePath: 'chatbot',
      },
    ],
  ],
};
```

## API Usage Examples

### Book Mode Query
```bash
curl -X POST http://localhost:8000/api/v1/chat/book \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the key features of ROS 2?",
    "session_id": "session-123"
  }'
```

### Selection Mode Query
```bash
curl -X POST http://localhost:8000/api/v1/chat/selection \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What does this text say about real-time support?",
    "selected_text": "ROS 2 introduces several key improvements over ROS 1. First, it provides better real-time support with improved timing guarantees...",
    "session_id": "session-123"
  }'
```

## Testing the Integration

### 1. Health Check
```bash
curl http://localhost:8000/api/v1/health
```

### 2. Basic Query Test
```bash
curl -X POST http://localhost:8000/api/v1/chat/book \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is this book about?",
    "session_id": "test-session"
  }'
```

### 3. Statistics Check
```bash
curl http://localhost:8000/api/v1/stats
```

## Production Deployment

### Backend Deployment
1. Set up production environment variables
2. Deploy to cloud platform (AWS, GCP, Azure, or Vercel)
3. Configure domain and SSL certificate
4. Set up monitoring and logging

### Frontend Integration
1. Update API endpoints to production URL
2. Build and deploy Docusaurus site
3. Test chatbot functionality end-to-end

## Troubleshooting

### Common Issues
- **Qdrant Connection**: Verify URL and API key in environment variables
- **Embedding Generation**: Check Qwen model access and API keys
- **Database Connection**: Ensure Neon connection string is correct
- **LLM Requests**: Verify OpenAI Router configuration

### Debugging Tips
- Enable debug logging: `LOG_LEVEL=DEBUG uvicorn src.main:app --reload`
- Check Qdrant collection: Verify document chunks are stored correctly
- Monitor API responses: Use browser dev tools to inspect requests/responses