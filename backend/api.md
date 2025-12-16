# RAG Chatbot API Documentation

## Overview

The RAG Chatbot API provides endpoints for interacting with the book content using natural language queries. The API supports both Book Mode (searching the entire book corpus) and Selection Mode (using user-selected text as context).

## Base URL

All API endpoints are prefixed with `/api/v1`.

## Authentication

The current implementation does not require authentication. For production use, implement API key authentication.

## Endpoints

### POST /chat/book

Submit a question for Book Mode Q&A (searches entire book corpus).

#### Request Body

```json
{
  "question": "What is ROS 2 and how does it differ from ROS 1?",
  "session_id": "session-12345",
  "user_id": "user-67890"
}
```

#### Request Parameters

- `question` (string, required): The user's question about the book content (10-500 characters)
- `session_id` (string, optional): Session identifier for tracking conversation history
- `user_id` (string, optional): User identifier if authenticated

#### Response

```json
{
  "response_id": "resp-abc123",
  "answer": "ROS 2 is the next generation of the Robot Operating System...",
  "citations": [
    {
      "module": "Module 1: The Robotic Nervous System (ROS 2)",
      "chapter": "Introduction to ROS 2",
      "section": "ROS 1 vs ROS 2",
      "url": "/docs/module-1-ros2/index"
    }
  ],
  "confidence_score": 0.89,
  "was_refused": false,
  "processing_time_ms": 1250
}
```

#### Response Fields

- `response_id` (string): Unique identifier for the response
- `answer` (string): The generated answer (null if refused)
- `citations` (array): List of source citations
- `confidence_score` (number): Confidence level (0.0-1.0)
- `was_refused` (boolean): Whether the system refused to answer
- `refusal_reason` (string): Reason for refusal (if applicable)
- `processing_time_ms` (number): Time taken to process the request

### POST /chat/selection

Submit a question for Selection Mode Q&A (uses only user-selected text).

#### Request Body

```json
{
  "question": "What does this text say about real-time support?",
  "selected_text": "ROS 2 introduces several key improvements over ROS 1. First, it provides better real-time support with improved timing guarantees...",
  "session_id": "session-12345",
  "user_id": "user-67890"
}
```

#### Request Parameters

- `question` (string, required): The user's question (10-500 characters)
- `selected_text` (string, required): The text selected by the user (10-2000 characters)
- `session_id` (string, optional): Session identifier
- `user_id` (string, optional): User identifier

#### Response

Same as Book Mode response.

### POST /ingest

Ingest and process book content from the docs/ folder.

#### Request Body

```json
{
  "force_reprocess": false,
  "document_paths": ["/docs/module-1-ros2/index.md", "/docs/module-1-ros2/nodes-topics-services.md"]
}
```

#### Request Parameters

- `force_reprocess` (boolean, optional): Whether to reprocess documents even if they already exist
- `document_paths` (array, optional): Specific document paths to process (if empty, processes all docs)

#### Response

```json
{
  "job_id": "ingest-job-xyz789",
  "status": "processing",
  "total_documents": 25,
  "processed_documents": 3,
  "failed_documents": 0,
  "message": "Ingestion started successfully"
}
```

### GET /health

Check the health status of the RAG service.

#### Response

```json
{
  "status": "healthy",
  "timestamp": "2025-12-16T10:30:00Z",
  "services": {
    "qdrant": "connected",
    "neon": "connected",
    "openai_router": "available"
  },
  "version": "1.0.0"
}
```

### GET /stats

Get usage statistics for the RAG service.

#### Response

```json
{
  "total_queries": 1250,
  "successful_queries": 1180,
  "refused_queries": 70,
  "avg_response_time_ms": 1150,
  "active_sessions": 25,
  "last_24h_queries": 180,
  "accuracy_rate": 0.87
}
```

## Error Handling

The API returns appropriate HTTP status codes:

- `200`: Success
- `400`: Bad Request (invalid input)
- `429`: Too Many Requests (rate limited)
- `500`: Internal Server Error

Error responses include a message field with details:

```json
{
  "detail": "Error message describing the issue"
}
```

## Rate Limits

- General endpoints: 100 requests per minute per IP
- Ingestion endpoints: 50 requests per minute per IP

## Response Format

All responses follow the same structure with appropriate fields based on the endpoint.