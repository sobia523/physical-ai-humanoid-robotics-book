import unittest
import os
import sys
from unittest.mock import Mock, patch, MagicMock
import tempfile
import requests
from datetime import datetime

# Add the backend directory to the path so we can import main
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

import main

class TestRAGPipeline(unittest.TestCase):
    """Unit tests for the RAG Pipeline core functions."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Set up environment variables for testing
        os.environ['COHERE_API_KEY'] = 'test-key'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'
        os.environ['QDRANT_URL'] = 'https://test-qdrant-cluster.com'

    def tearDown(self):
        """Clean up after each test method."""
        # Remove test environment variables
        if 'COHERE_API_KEY' in os.environ:
            del os.environ['COHERE_API_KEY']
        if 'QDRANT_API_KEY' in os.environ:
            del os.environ['QDRANT_API_KEY']
        if 'QDRANT_URL' in os.environ:
            del os.environ['QDRANT_URL']

    def test_get_config(self):
        """Test that get_config returns expected configuration."""
        config = main.get_config()

        self.assertIsNotNone(config)
        self.assertIn('CHUNK_SIZE', config)
        self.assertIn('OVERLAP_SIZE', config)
        self.assertIn('SITEMAP_URL', config)
        self.assertIn('QDRANT_COLLECTION_NAME', config)
        self.assertIn('COHERE_MODEL', config)
        self.assertEqual(config['CHUNK_SIZE'], 800)
        self.assertEqual(config['OVERLAP_SIZE'], 200)

    def test_count_tokens(self):
        """Test that count_tokens returns correct token counts."""
        test_text = "This is a test sentence."
        token_count = main.count_tokens(test_text)

        # The exact token count may vary depending on the tokenizer, but it should be reasonable
        self.assertIsInstance(token_count, int)
        self.assertGreater(token_count, 0)

    def test_is_valid_url(self):
        """Test URL validation function."""
        # Valid URLs
        self.assertTrue(main.is_valid_url("https://example.com"))
        self.assertTrue(main.is_valid_url("http://example.com/path"))
        self.assertTrue(main.is_valid_url("https://subdomain.example.com:8080/path?query=value"))

        # Invalid URLs
        self.assertFalse(main.is_valid_url("not-a-url"))
        self.assertFalse(main.is_valid_url(""))
        self.assertFalse(main.is_valid_url(None))
        self.assertFalse(main.is_valid_url("ftp://example.com"))  # Though this might be valid depending on implementation

    def test_sanitize_url(self):
        """Test URL sanitization function."""
        # Test URL with uppercase scheme and fragment
        original_url = "HTTP://EXAMPLE.COM/PAGE#section"
        sanitized = main.sanitize_url(original_url)

        self.assertEqual(sanitized, "http://example.com/PAGE")  # Fragment should be removed

        # Test normal URL
        normal_url = "https://example.com/page"
        sanitized = main.sanitize_url(normal_url)
        self.assertEqual(sanitized, "https://example.com/page")

    def test_extract_content_from_html(self):
        """Test HTML content extraction."""
        sample_html = """
        <html>
            <head><title>Test Page</title></head>
            <body>
                <h1>Introduction</h1>
                <p>This is the introduction paragraph.</p>
                <h2>Section 1</h2>
                <p>This is the first section paragraph.</p>
                <p>This is another paragraph in section 1.</p>
                <h2>Section 2</h2>
                <p>This is the second section paragraph.</p>
            </body>
        </html>
        """

        result = main.extract_content_from_html(sample_html, "https://example.com/test")

        self.assertIsNotNone(result)
        self.assertEqual(result['title'], 'Test Page')
        self.assertEqual(len(result['sections']), 3)  # Introduction, Section 1, Section 2
        self.assertGreater(len(result['paragraphs']), 0)

    def test_clean_extracted_content(self):
        """Test content cleaning function."""
        raw_content = {
            'title': '  Test Title  ',
            'sections': [
                {
                    'title': '  Section 1  ',
                    'content': '  This is section content with extra spaces.  '
                }
            ],
            'paragraphs': [
                '  Paragraph with extra spaces  ',
                'Another paragraph    with    multiple    spaces'
            ],
            'url': 'https://example.com/test'
        }

        cleaned = main.clean_extracted_content(raw_content)

        self.assertEqual(cleaned['title'], 'Test Title')
        self.assertEqual(cleaned['sections'][0]['title'], 'Section 1')
        self.assertEqual(cleaned['sections'][0]['content'], 'This is section content with extra spaces.')
        self.assertEqual(cleaned['paragraphs'][0], 'Paragraph with extra spaces')

    def test_chunk_text(self):
        """Test text chunking function."""
        # Create a text that will definitely exceed the chunk size
        long_text = "This is a sentence. " * 100  # 200 words, should be > 800 tokens

        # Test with small chunk size to ensure chunking occurs
        chunks = main.chunk_text(long_text, chunk_size=50, overlap_size=10)

        self.assertGreater(len(chunks), 1)  # Should be split into multiple chunks
        self.assertLessEqual(len(chunks[0].split()), 50)  # First chunk should respect size limit

        # Test with text shorter than chunk size
        short_text = "Short text."
        chunks = main.chunk_text(short_text, chunk_size=50, overlap_size=10)

        self.assertEqual(len(chunks), 1)  # Should not be chunked
        self.assertEqual(chunks[0], short_text)

    def test_validate_embeddings_format(self):
        """Test embedding format validation."""
        # Valid embeddings
        valid_embeddings = [
            [0.1, 0.2, 0.3] * 341 + [0.4],  # 1024-dim vector
            [0.5] * 1024  # Another 1024-dim vector
        ]

        result = main.validate_embeddings_format(valid_embeddings, expected_dim=1024)
        self.assertTrue(result)

        # Invalid embeddings (wrong dimension)
        invalid_embeddings = [
            [0.1, 0.2, 0.3]  # Only 3 dimensions instead of 1024
        ]

        result = main.validate_embeddings_format(invalid_embeddings, expected_dim=1024)
        self.assertFalse(result)

        # Empty embeddings
        result = main.validate_embeddings_format([], expected_dim=1024)
        self.assertFalse(result)

    def test_check_embedding_quality(self):
        """Test embedding quality checking function."""
        sample_embeddings = [
            [0.1, 0.2, 0.3] * 341 + [0.4],  # 1024-dim vector
            [0.5] * 1024  # Another 1024-dim vector
        ]

        quality_stats = main.check_embedding_quality(sample_embeddings)

        self.assertIsNotNone(quality_stats)
        self.assertEqual(quality_stats['count'], 2)
        self.assertEqual(quality_stats['dimension'], 1024)
        self.assertFalse(quality_stats['has_nan'])
        self.assertFalse(quality_stats['has_inf'])

    def test_generate_url_coverage_report(self):
        """Test URL coverage report generation."""
        processed_urls = ["https://example.com/page1", "https://example.com/page2"]
        all_urls = ["https://example.com/page1", "https://example.com/page2", "https://example.com/page3"]

        report = main.generate_url_coverage_report(processed_urls, all_urls)

        self.assertIsNotNone(report)
        self.assertIn("URL COVERAGE REPORT", report)
        self.assertIn("Total URLs in Sitemap: 3", report)
        self.assertIn("Processed URLs: 2", report)
        self.assertIn("Missing URLs: 1", report)

    def test_verify_success_criteria(self):
        """Test success criteria verification."""
        # Create mock Qdrant client
        mock_qdrant_client = Mock()
        mock_collection_info = Mock()
        mock_collection_info.points_count = 100
        mock_qdrant_client.get_collection.return_value = mock_collection_info

        # Test with successful stats
        stats = {
            'processed_urls': 5,
            'failed_urls': 0,
            'total_chunks': 250,  # Above threshold
        }

        results = main.verify_success_criteria(stats, mock_qdrant_client)

        # Check that criteria are properly evaluated
        self.assertIsNotNone(results)
        self.assertIn('criteria_met', results)

class TestIntegration(unittest.TestCase):
    """Integration tests for the pipeline components."""

    def test_chunk_content_by_sections(self):
        """Test the complete content chunking pipeline."""
        sample_content = {
            'title': 'Test Content',
            'sections': [
                {
                    'title': 'Section 1',
                    'content': 'This is the content of section 1. ' * 50
                }
            ],
            'paragraphs': [
                'This is a paragraph. ' * 20
            ],
            'url': 'https://example.com/test'
        }

        chunks = main.chunk_content_by_sections(sample_content)

        self.assertGreater(len(chunks), 0)
        for chunk in chunks:
            self.assertIn('chunk_text', chunk)
            self.assertIn('url', chunk)
            self.assertIn('section_title', chunk)

if __name__ == '__main__':
    unittest.main()