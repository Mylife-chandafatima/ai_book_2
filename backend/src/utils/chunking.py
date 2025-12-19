import re
import logging
from typing import List, Dict, Any
from dataclasses import dataclass
from uuid import uuid4


logger = logging.getLogger(__name__)


@dataclass
class DocumentChunkData:
    """Represents a chunk of a document with metadata"""
    content: str
    module: str
    chapter: str
    section: str
    position: int
    metadata: Dict[str, Any]


def chunk_markdown_content(content: str, document_path: str, max_chunk_size: int = 600) -> List[DocumentChunkData]:
    """
    Split markdown content into chunks based on headings while preserving document structure.

    Args:
        content: The markdown content to chunk
        document_path: Path of the source document
        max_chunk_size: Maximum size of each chunk in tokens/characters

    Returns:
        List of DocumentChunkData objects
    """
    chunks = []

    # Extract module and chapter from document path
    path_parts = document_path.replace('/docs/', '').split('/')
    module = path_parts[0] if path_parts else "Unknown Module"

    # Extract chapter name from file name
    chapter = path_parts[-1].replace('.md', '').replace('.mdx', '') if path_parts else "Unknown Chapter"

    # Split content by headings
    heading_pattern = r'^(#{1,6})\s+(.+)$'
    lines = content.split('\n')

    current_section = "Introduction"
    current_content = []
    position = 0

    for line in lines:
        # Check if this line is a heading
        match = re.match(heading_pattern, line.strip())
        if match:
            # If we have accumulated content, save it as a chunk
            if current_content and len(' '.join(current_content).strip()) > 0:
                chunk_text = '\n'.join(current_content).strip()
                if len(chunk_text) > 0:
                    chunks.append(
                        DocumentChunkData(
                            content=chunk_text,
                            module=module,
                            chapter=chapter,
                            section=current_section,
                            position=position,
                            metadata={"document_path": document_path}
                        )
                    )
                    position += 1

            # Start a new section with this heading
            current_section = match.group(2).strip()
            current_content = [line]
        else:
            current_content.append(line)

    # Don't forget the last chunk
    if current_content and len(' '.join(current_content).strip()) > 0:
        chunk_text = '\n'.join(current_content).strip()
        if len(chunk_text) > 0:
            chunks.append(
                DocumentChunkData(
                    content=chunk_text,
                    module=module,
                    chapter=chapter,
                    section=current_section,
                    position=position,
                    metadata={"document_path": document_path}
                )
            )

    # If chunks are too large, split them further
    final_chunks = []
    for i, chunk in enumerate(chunks):
        if len(chunk.content) > max_chunk_size:
            # Split large chunks by paragraphs
            paragraphs = chunk.content.split('\n\n')
            current_sub_chunk = []
            current_size = 0

            for para in paragraphs:
                if current_size + len(para) <= max_chunk_size:
                    current_sub_chunk.append(para)
                    current_size += len(para)
                else:
                    # Save current sub-chunk and start a new one
                    if current_sub_chunk:
                        final_chunks.append(
                            DocumentChunkData(
                                content='\n\n'.join(current_sub_chunk),
                                module=chunk.module,
                                chapter=chunk.chapter,
                                section=f"{chunk.section} (Part {len(final_chunks) + 1})",
                                position=position,
                                metadata=chunk.metadata
                            )
                        )
                        position += 1

                    # Start new sub-chunk with current paragraph if it's not too large
                    if len(para) <= max_chunk_size:
                        current_sub_chunk = [para]
                        current_size = len(para)
                    else:
                        # If a single paragraph is too large, split it by sentences
                        sentences = re.split(r'[.!?]+', para)
                        temp_para = ""
                        for sentence in sentences:
                            if len(temp_para + sentence) <= max_chunk_size:
                                temp_para += sentence + ". "
                            else:
                                if temp_para:
                                    final_chunks.append(
                                        DocumentChunkData(
                                            content=temp_para.strip(),
                                            module=chunk.module,
                                            chapter=chunk.chapter,
                                            section=f"{chunk.section} (Part {len(final_chunks) + 1})",
                                            position=position,
                                            metadata=chunk.metadata
                                        )
                                    )
                                    position += 1
                                temp_para = sentence + ". "

                        if temp_para.strip():
                            current_sub_chunk = [temp_para.strip()]
                            current_size = len(temp_para.strip())

            # Add remaining content if any
            if current_sub_chunk:
                final_chunks.append(
                    DocumentChunkData(
                        content='\n\n'.join(current_sub_chunk),
                        module=chunk.module,
                        chapter=chunk.chapter,
                        section=chunk.section,
                        position=position,
                        metadata=chunk.metadata
                    )
                )
                position += 1
        else:
            final_chunks.append(chunk)

    logger.info(f"Chunked document {document_path} into {len(final_chunks)} chunks")
    return final_chunks


def calculate_chunk_overlap(chunk1: str, chunk2: str) -> float:
    """
    Calculate overlap between two chunks as a ratio of common words.

    Args:
        chunk1: First chunk text
        chunk2: Second chunk text

    Returns:
        Overlap ratio from 0.0 to 1.0
    """
    words1 = set(chunk1.lower().split())
    words2 = set(chunk2.lower().split())

    intersection = words1.intersection(words2)
    union = words1.union(words2)

    if not union:
        return 0.0

    return len(intersection) / len(union)