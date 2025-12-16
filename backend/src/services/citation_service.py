import logging
from typing import List
from urllib.parse import quote
from src.models.response import Citation


logger = logging.getLogger(__name__)


class CitationService:
    """Service for handling citations and generating proper book navigation links"""

    def __init__(self):
        pass

    def generate_book_url(self, module: str, chapter: str) -> str:
        """
        Generate proper book navigation URL based on module and chapter.

        Args:
            module: Module name
            chapter: Chapter name

        Returns:
            Properly formatted URL for the book section
        """
        try:
            # Normalize module and chapter names for URL construction
            # Convert to lowercase and replace spaces with hyphens
            normalized_module = self.normalize_for_url(module)
            normalized_chapter = self.normalize_for_url(chapter)

            # Map common module names to actual directory names in the book structure
            module_mapping = {
                "module-1-ros2-foundations": "module-1-ros2",
                "module-2-simulation": "module-3-nvidia-isaac",
                "module-3-hardware-foundations": "modules/module-3",
                "module-4-vla-foundations": "modules/module-2",
                "module-5-advanced-ai-control": "modules/module-4",
                "module-6-humanoid-design": "modules/module-6"
            }

            # Use mapping if available, otherwise use normalized name
            actual_module = module_mapping.get(normalized_module, normalized_module)

            # Construct the URL based on the book structure
            if actual_module and normalized_chapter:
                # Try to construct a meaningful URL based on the book structure
                # Check if it's a common pattern in the docs structure
                if actual_module.startswith("module-"):
                    url = f"/docs/{actual_module}/{normalized_chapter}"
                elif actual_module.startswith("modules/"):
                    # Handle nested modules
                    url = f"/docs/{actual_module}/{normalized_chapter}"
                else:
                    # General case
                    url = f"/docs/{actual_module}/{normalized_chapter}"

                return url
            else:
                # Fallback to a general documentation URL
                return "/docs/intro"

        except Exception as e:
            logger.error(f"Error generating book URL for module '{module}', chapter '{chapter}': {str(e)}")
            return "/docs/intro"  # Fallback URL

    def enhance_citation_with_detailed_reference(self, module: str, chapter: str, section: str, document_path: str = "") -> Citation:
        """
        Enhance citation with detailed reference information.

        Args:
            module: Module name
            chapter: Chapter name
            section: Section name
            document_path: Document path for additional context

        Returns:
            Enhanced Citation object
        """
        url = self.generate_book_url(module, chapter)

        return Citation(
            module=module,
            chapter=chapter,
            section=section,
            url=url
        )

    def format_citation_for_frontend(self, citation: Citation) -> dict:
        """
        Format citation specifically for frontend display.

        Args:
            citation: Citation object to format

        Returns:
            Dictionary with citation formatted for frontend
        """
        return {
            "module": citation.module,
            "chapter": citation.chapter,
            "section": citation.section,
            "url": citation.url,
            "display_text": f"{citation.module}, {citation.chapter}"
        }

    def normalize_for_url(self, text: str) -> str:
        """
        Normalize text for use in URLs (lowercase, replace spaces with hyphens, etc.).

        Args:
            text: Text to normalize

        Returns:
            Normalized text suitable for URLs
        """
        import re

        # Convert to lowercase
        text = text.lower()

        # Replace spaces and common separators with hyphens
        text = re.sub(r'[ _/\\]', '-', text)

        # Remove special characters but keep alphanumeric and hyphens
        text = re.sub(r'[^a-z0-9-]', '', text)

        # Replace multiple hyphens with single hyphen
        text = re.sub(r'-+', '-', text)

        # Remove leading/trailing hyphens
        text = text.strip('-')

        return text

    def format_citations(self, citations: List[Citation]) -> List[dict]:
        """
        Format citations for API response.

        Args:
            citations: List of Citation objects

        Returns:
            List of formatted citation dictionaries
        """
        formatted_citations = []
        for citation in citations:
            formatted_citations.append({
                "module": citation.module,
                "chapter": citation.chapter,
                "section": citation.section,
                "url": citation.url
            })
        return formatted_citations

    def create_citation_from_chunk(self, module: str, chapter: str, section: str, document_path: str = "") -> Citation:
        """
        Create a citation object from chunk information.

        Args:
            module: Module name
            chapter: Chapter name
            section: Section name
            document_path: Optional document path for additional context

        Returns:
            Citation object
        """
        url = self.generate_book_url(module, chapter)

        return Citation(
            module=module,
            chapter=chapter,
            section=section,
            url=url
        )

    def validate_citation_format(self, citation: Citation) -> bool:
        """
        Validate that a citation has the proper format and required fields.

        Args:
            citation: Citation object to validate

        Returns:
            True if valid, False otherwise
        """
        if not citation.module or not citation.module.strip():
            return False

        if not citation.chapter or not citation.chapter.strip():
            return False

        if not citation.url or not citation.url.startswith('/'):
            return False

        return True

    def generate_module_chapter_reference(self, module: str, chapter: str) -> str:
        """
        Generate a human-readable module-chapter reference.

        Args:
            module: Module name
            chapter: Chapter name

        Returns:
            Formatted reference string
        """
        return f"{module}, Chapter: {chapter}"

    def enhance_citations_with_metadata(self, citations: List[Citation], metadata: dict) -> List[Citation]:
        """
        Enhance citations with additional metadata.

        Args:
            citations: List of citations to enhance
            metadata: Additional metadata to include

        Returns:
            List of enhanced Citation objects
        """
        enhanced_citations = []
        for citation in citations:
            # In a real implementation, you might add additional fields to the citation
            # For now, we'll just return the original citation
            enhanced_citations.append(citation)

        return enhanced_citations