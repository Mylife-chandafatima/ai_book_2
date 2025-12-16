# Research: Integrated RAG Chatbot for Docusaurus Book

## Architecture Sketch

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Docusaurus Book Frontend                         │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌─────────────────────────────────────────────┐ │
│  │   Chat Widget   │◄──►│    User Selection (Optional)              │ │
│  │                 │    │                                             │ │
│  │ - Book Mode     │    │ - Selected Text Context                     │ │
│  │ - Selection Mode│    │ - Mode Toggle                               │ │
│  │ - Citations     │    │                                             │ │
│  └─────────────────┘    └─────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼ HTTP/JSON
┌─────────────────────────────────────────────────────────────────────────┐
│                        FastAPI Backend                                  │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────────┐  │
│  │   Ingestion     │    │   Retrieval     │    │   Generation        │  │
│  │   Service       │    │   Service       │    │   Service           │  │
│  │                 │    │                 │    │                     │  │
│  │ - Parse docs/   │    │ - Vector search │    │ - OpenAI Router   │  │
│  │ - Chunk content │    │ - Qdrant Cloud  │    │ - Context prompt  │  │
│  │ - Store vectors │    │ - Book vs Sel   │    │ - Citations       │  │
│  │ - Meta to Neon  │    │ - Refusal logic │    │ - Grounding check │  │
│  └─────────────────┘    └─────────────────┘    └─────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                ┌───────────────────┼───────────────────┐
                ▼                   ▼                   ▼
        ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
        │   Qdrant Cloud  │ │  Neon Postgres  │ │  OpenAI Router  │
        │   (Vectors)     │ │   (Metadata)    │ │   (LLM)         │
        │                 │ │                 │ │                 │
        │ - Embeddings    │ │ - Doc refs      │ │ - Qwen embeds   │ │
        │ - Similarity    │ │ - Module/chap   │ │ - Response gen  │ │
        │ - Search        │ │ - Relationships │ │ - Model routing │ │
        └─────────────────┘ └─────────────────┘ └─────────────────┘
```

### Request Flow - Book Mode:
1. User submits question in Book Mode
2. Frontend sends query to /api/v1/chat/book
3. Retrieval Service performs vector search across entire corpus
4. Generation Service creates context prompt with retrieved chunks
5. OpenAI Router generates response with source citations
6. Frontend displays response with clickable citations

### Request Flow - Selection Mode:
1. User selects text and submits question in Selection Mode
2. Frontend sends selected text + query to /api/v1/chat/selection
3. Retrieval Service processes selected text and performs targeted search
4. Generation Service creates context prompt with selected text
5. OpenAI Router generates response (only from selected context)
6. Frontend displays response with source attribution

## Section Structure

### Backend Service Layout:
- **Ingestion Service**: Handles document parsing, chunking, embedding, and storage
- **Retrieval Service**: Performs vector searches and context retrieval
- **Generation Service**: Formats prompts and calls LLM service
- **Citation Service**: Generates proper source references

### API Endpoints:
- `POST /api/v1/chat/book` - Book Mode Q&A
- `POST /api/v1/chat/selection` - Selection Mode Q&A
- `POST /api/v1/ingest` - Content ingestion
- `GET /api/v1/health` - Health checks
- `GET /api/v1/stats` - Usage statistics

### Vector Storage (Qdrant Cloud) vs Relational Storage (Neon Postgres):
- **Qdrant**: Stores document embeddings, content text, and basic metadata for similarity search
- **Neon**: Stores document relationships, module/chapter references, user data, and complex metadata

### Frontend Integration Components:
- **ChatbotWidget**: React component with message history, input area, mode toggle
- **CitationRenderer**: Displays source links that navigate to specific book sections
- **ModeSelector**: Toggle between Book Mode and Selection Mode

## Research Approach

### RAG Best Practices:
- Use semantic chunking with overlap to preserve context across boundaries
- Implement re-ranking for improved retrieval accuracy
- Apply query expansion techniques to improve search relevance
- Use hybrid search combining semantic and keyword matching

### Chunk Size and Overlap Strategy:
- Base chunk size on document structure (sections between headings)
- Target 200-400 tokens per chunk for optimal context window usage
- Use 50-100 token overlap to maintain context continuity
- Preserve document hierarchy in chunk metadata

### Grounding and Citation Methods:
- Include source document ID, section, and position in context
- Use exact text matching to verify citation accuracy
- Implement confidence scoring to determine when to refuse responses
- Format citations as "Module X, Chapter Y" with direct links

### Handling Selection-Only Constraints:
- Process selected text as primary context with zero tolerance for external information
- Implement strict context filtering to exclude non-selected content
- Use specialized prompt engineering to focus on provided context only
- Validate responses against selection boundaries

### OpenAI Router Usage Patterns:
- Leverage router for model-agnostic LLM calls
- Implement proper error handling for LLM failures
- Use structured output formats for consistent response parsing
- Apply rate limiting and retry logic for reliability

## Decisions Needing Documentation

### Chunking Strategy Options:
- **Option A**: Heading-aware chunking (recommended)
  - Pros: Preserves document structure, natural boundaries
  - Cons: May create uneven chunk sizes
- **Option B**: Fixed token length chunking
  - Pros: Consistent processing, predictable performance
  - Cons: May split related content
- **Option C**: Hybrid approach (heading boundaries + token limits)
  - Pros: Best of both approaches
  - Cons: More complex implementation

### Qwen Embedding Model Choice:
- **Qwen2-7B-Embedding**: Good balance of quality and cost
- **Qwen2-72B-Embedding**: Higher quality, higher cost
- **Qwen2-1.5B-Embedding**: Lower quality, lowest cost

### Metadata Schema Design:
- Document-level: module, chapter, section, title, path
- Chunk-level: position, parent document, semantic boundaries
- Relationship: cross-references between related content

### Refusal Logic When Context is Missing:
- Set similarity threshold (e.g., 0.7 cosine similarity)
- Implement multiple retrieval attempts with query variations
- Use confidence scoring to determine response validity
- Provide clear refusal messages with explanations

### Book-Wide vs Selection-Based Retrieval:
- Book-wide: Vector search across entire corpus with relevance ranking
- Selection-based: Focus on provided text context with strict boundaries
- Implement separate retrieval strategies for each mode

## Testing Strategy

### Acceptance Criteria:
- 95% of questions receive accurate, cited responses within 5 seconds
- System refuses at least 90% of queries with no relevant content
- 90% of users find needed information within 2 attempts
- Response accuracy exceeds 85% in manual validation

### Retrieval Accuracy Validation:
- Test with questions targeting specific book sections
- Verify citation accuracy against source documents
- Measure precision and recall of vector search
- Validate cross-module reference accuracy

### Hallucination Prevention Tests:
- Submit queries with no relevant book content
- Verify system refuses rather than fabricating answers
- Test edge cases with similar but incorrect information
- Validate response grounding in provided context

### Selection-Mode Edge Cases:
- Empty or minimal selected text
- Multiple disconnected text selections
- Selection spanning multiple chapters/modules
- Very long text selections exceeding context limits