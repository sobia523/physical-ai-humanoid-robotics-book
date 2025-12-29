"""
Unit tests for the RAG Chatbot Agent retrieval functionality.

This module contains tests for the Qdrant retrieval functions
including connection, embedding generation, chunk retrieval, and context formatting.
"""
import pytest
from unittest.mock import Mock, patch
from backend.utils.qdrant_helper import (
    connect_qdrant,
    build_query_embedding,
    retrieve_chunks,
    format_context
)


class TestConnectQdrant:
    """Test cases for the connect_qdrant function."""

    def test_connect_qdrant_success(self):
        """Test successful connection to Qdrant."""
        with patch('qdrant_client.QdrantClient') as mock_client:
            mock_client_instance = Mock()
            mock_client.return_value = mock_client_instance

            # Mock the actual connection
            result = connect_qdrant()

            assert result == mock_client_instance
            mock_client.assert_called_once()


class TestBuildQueryEmbedding:
    """Test cases for the build_query_embedding function."""

    def test_build_query_embedding_success(self):
        """Test successful embedding generation."""
        with patch('cohere.Client') as mock_cohere:
            mock_cohere_instance = Mock()
            mock_cohere_instance.embed.return_value = Mock(embeddings=[[0.1, 0.2, 0.3]])
            mock_cohere.return_value = mock_cohere_instance

            result = build_query_embedding("test text")

            assert isinstance(result, list)
            assert len(result) == 3  # Based on mocked response
            mock_cohere_instance.embed.assert_called_once()


class TestRetrieveChunks:
    """Test cases for the retrieve_chunks function."""

    def test_retrieve_chunks_success(self):
        """Test successful chunk retrieval."""
        with patch('backend.utils.qdrant_helper.connect_qdrant') as mock_connect:
            # Mock the Qdrant client and search results
            mock_client = Mock()
            mock_connect.return_value = mock_client

            mock_search_result = [
                Mock(id="test_id", payload={"content": "test content", "url": "test_url"}, score=0.8)
            ]
            mock_client.search.return_value = mock_search_result

            result = retrieve_chunks([0.1, 0.2, 0.3], top_k=1)

            assert len(result) == 1
            assert result[0]['id'] == "test_id"
            assert result[0]['content'] == "test content"
            assert result[0]['url'] == "test_url"
            assert result[0]['score'] == 0.8


class TestFormatContext:
    """Test cases for the format_context function."""

    def test_format_context_with_results(self):
        """Test context formatting with results."""
        test_results = [
            {
                "id": "test_id",
                "content": "test content",
                "url": "test_url",
                "module": "test_module",
                "chapter": "test_chapter",
                "heading": "test_heading",
                "score": 0.8
            }
        ]

        result = format_context(test_results)

        assert "Relevant information from the book:" in result
        assert "Source 1:" in result
        assert "test_url" in result
        assert "test_module" in result
        assert "test content" in result

    def test_format_context_empty_results(self):
        """Test context formatting with no results."""
        result = format_context([])

        assert result == "No relevant content found in the knowledge base."


if __name__ == "__main__":
    pytest.main([__file__])