# Implementation Plan: Integrated RAG Chatbot for Docusaurus Book

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-16 | **Spec**: [specs/001-rag-chatbot/spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of an integrated RAG chatbot for a Docusaurus book that supports both Book Mode (entire book corpus) and Selection Mode (user-selected text) Q&A functionality. The system uses Qwen embeddings for vectorization, Qdrant Cloud for vector storage, Neon Serverless Postgres for metadata, FastAPI backend, and OpenAI Router for LLM inference, with strict grounding to prevent hallucinations.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript for frontend integration
**Primary Dependencies**: FastAPI, Qwen embedding models, OpenAI Router, Qdrant client, Neon Postgres driver, Docusaurus plugins
**Storage**: Qdrant vector database for embeddings, Neon Serverless Postgres for metadata and document references
**Testing**: pytest for backend, Jest for frontend components, integration tests for RAG pipeline
**Target Platform**: Linux server for backend, web browser for frontend integration
**Project Type**: Web application (backend API + Docusaurus frontend integration)
**Performance Goals**: <3 second response time, 95% reliability under 100 concurrent users, 95% accurate answer rate
**Constraints**: Free-tier compatible, no hallucinations, proper source citations, sub-5s response times
**Scale/Scope**: 1000+ book pages, 100+ concurrent users, 90%+ success rate for information retrieval

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Accuracy & Verifiability**: RAG system must provide source citations for all responses (PASSED - spec requirement FR-009)
- **Clarity & Audience Awareness**: System must integrate seamlessly with Docusaurus deployment (PASSED - spec requirement FR-010)
- **Reproducibility & Rigor**: All components must be testable and verifiable (PASSED - success criteria SC-004)
- **AI & Tool Integration**: RAG chatbot integration instructions must be functional (PASSED - spec requirement FR-010)
- **Ethics & Safety**: System must prevent hallucinations and provide accurate information (PASSED - spec requirement FR-013)

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── document_chunk.py      # Document chunk entity with metadata
│   │   ├── user_query.py          # Query entity with mode context
│   │   └── response.py            # Response entity with citations
│   ├── services/
│   │   ├── ingestion_service.py   # Content ingestion and chunking
│   │   ├── retrieval_service.py   # Vector search and context retrieval
│   │   ├── generation_service.py  # LLM response generation
│   │   └── citation_service.py    # Source citation formatting
│   ├── api/
│   │   ├── v1/
│   │   │   ├── chat.py           # Chat endpoints
│   │   │   ├── ingestion.py      # Content ingestion endpoints
│   │   │   └── health.py         # Health check endpoints
│   │   └── deps.py               # Dependency injection
│   ├── config/
│   │   ├── settings.py           # Configuration settings
│   │   └── database.py           # Database connection setup
│   └── utils/
│       ├── chunking.py           # Text chunking utilities
│       └── validators.py         # Input validation utilities
└── tests/
    ├── unit/
    ├── integration/
    └── contract/

src/
├── components/
│   └── ChatbotWidget/            # Docusaurus React component for chatbot
├── pages/                        # Updated homepage with chatbot integration
└── css/                          # Custom styles for chatbot widget

docs/                             # Book content (unchanged per spec FR-001)
├── ...
└── ...
```

**Structure Decision**: Web application structure chosen with separate backend API service and Docusaurus frontend integration. The backend handles all RAG operations (ingestion, retrieval, generation) while the frontend provides an embedded chatbot widget within the Docusaurus book interface.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [N/A] | [All constitution checks passed] |
