import unittest
from src.utils.chunking import chunk_markdown_content


class TestChunking(unittest.TestCase):
    """Unit tests for chunking utilities"""

    def test_basic_chunking(self):
        """Test basic markdown content chunking"""
        content = "# Introduction\n\nThis is the introduction to the chapter.\n\n## Section 1\n\nThis is the first section."
        document_path = "/docs/test-module/test-chapter.md"

        chunks = chunk_markdown_content(content, document_path)

        self.assertGreater(len(chunks), 0)
        self.assertEqual(chunks[0].module, "test-module")
        self.assertEqual(chunks[0].chapter, "test-chapter")
        self.assertIn("Introduction", chunks[0].section)

    def test_chunking_with_large_content(self):
        """Test chunking of large content that should be split"""
        # Create content larger than max_chunk_size
        large_content = "# Test\n\n" + "word " * 700  # This should exceed default max size
        document_path = "/docs/test-module/test-chapter.md"

        chunks = chunk_markdown_content(content=large_content, document_path=document_path, max_chunk_size=600)

        # Should have been split into multiple chunks
        self.assertGreater(len(chunks), 1)

    def test_chunking_preserves_structure(self):
        """Test that chunking preserves document structure"""
        content = "# Main Title\n\nIntro content\n\n## Subsection 1\n\nSubsection content\n\n## Subsection 2\n\nMore content"
        document_path = "/docs/test-module/test-chapter.md"

        chunks = chunk_markdown_content(content, document_path)

        # Should have chunks for each section
        self.assertGreater(len(chunks), 1)

        # Check that section names are preserved
        section_names = [chunk.section for chunk in chunks]
        self.assertIn("Main Title", section_names)
        self.assertIn("Subsection 1", section_names)
        self.assertIn("Subsection 2", section_names)


if __name__ == '__main__':
    unittest.main()