import unittest
from src.utils.validators import validate_user_query, validate_document_chunk


class TestValidation(unittest.TestCase):
    """Unit tests for validation utilities"""

    def test_user_query_validation_valid(self):
        """Test validation of valid user queries"""
        result = validate_user_query("This is a valid question about the book.", "book")
        self.assertTrue(result.is_valid)
        self.assertIsNotNone(result.sanitized_query)

    def test_user_query_validation_short(self):
        """Test validation of too-short queries"""
        result = validate_user_query("Hi", "book")
        self.assertFalse(result.is_valid)
        self.assertIsNotNone(result.error_message)

    def test_user_query_validation_long(self):
        """Test validation of too-long queries"""
        long_query = "This is a very long question. " * 30  # Way too long
        result = validate_user_query(long_query, "book")
        self.assertFalse(result.is_valid)
        self.assertIsNotNone(result.error_message)

    def test_selection_mode_validation(self):
        """Test validation for selection mode"""
        result = validate_user_query(
            "What does this text mean?",
            "selection",
            "Selected text content here"
        )
        self.assertTrue(result.is_valid)

    def test_selection_mode_validation_no_text(self):
        """Test validation for selection mode without selected text"""
        result = validate_user_query(
            "What does this text mean?",
            "selection",
            None
        )
        self.assertFalse(result.is_valid)
        self.assertIn("selected text", result.error_message.lower())

    def test_document_chunk_validation(self):
        """Test validation of document chunks"""
        result = validate_document_chunk(
            content="This is a valid chunk of content with sufficient length.",
            module="Test Module",
            chapter="Test Chapter"
        )
        self.assertTrue(result.is_valid)

    def test_document_chunk_validation_short(self):
        """Test validation of too-short document chunks"""
        result = validate_document_chunk(
            content="Hi",  # Too short
            module="Test Module",
            chapter="Test Chapter"
        )
        self.assertFalse(result.is_valid)
        self.assertIsNotNone(result.error_message)

    def test_document_chunk_validation_missing_module(self):
        """Test validation of document chunks with missing module"""
        result = validate_document_chunk(
            content="This is a valid chunk of content with sufficient length.",
            module="",  # Missing module
            chapter="Test Chapter"
        )
        self.assertFalse(result.is_valid)
        self.assertIsNotNone(result.error_message)


if __name__ == '__main__':
    unittest.main()