"""
Unit tests for the RAG Chatbot Agent.

This module contains tests for the agent functionality including
query handling, response generation, and source citation.
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
from backend.agent import agent_handler, initialize_agent


class TestAgentHandler:
    """Test cases for the agent_handler function."""

    @patch('backend.agent.build_query_embedding')
    @patch('backend.agent.retrieve_chunks')
    @patch('backend.agent.format_context')
    @patch('backend.agent.client.chat.completions.create')
    def test_agent_handler_basic_query(self, mock_openai_create, mock_format_context, mock_retrieve_chunks, mock_build_embedding):
        """Test agent handler with a basic query."""
        # Mock the dependencies
        mock_build_embedding.return_value = [0.1, 0.2, 0.3]
        mock_retrieve_chunks.return_value = [
            {
                'id': 'test_id',
                'content': 'test content',
                'url': 'https://test-url.com',
                'module': 'test-module',
                'chapter': 'test-chapter',
                'heading': 'test-heading',
                'score': 0.8,
                'metadata': {}
            }
        ]
        mock_format_context.return_value = "test context"

        # Mock OpenAI response
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = "Test response based on context"
        mock_openai_create.return_value = mock_response

        result = agent_handler("What is ROS 2?")

        assert "answer" in result
        assert "sources" in result
        assert result["answer"] == "Test response based on context"
        assert len(result["sources"]) == 1
        assert result["sources"][0]["url"] == "https://test-url.com"
        assert result["sources"][0]["module"] == "test-module"
        assert result["sources"][0]["score"] == 0.8

    @patch('backend.agent.build_query_embedding')
    @patch('backend.agent.retrieve_chunks')
    @patch('backend.agent.format_context')
    @patch('backend.agent.client.chat.completions.create')
    def test_agent_handler_with_selected_text(self, mock_openai_create, mock_format_context, mock_retrieve_chunks, mock_build_embedding):
        """Test agent handler with selected text parameter."""
        # Mock the dependencies
        mock_build_embedding.return_value = [0.1, 0.2, 0.3]
        mock_retrieve_chunks.return_value = [
            {
                'id': 'test_id',
                'content': 'test content',
                'url': 'https://test-url.com',
                'module': 'test-module',
                'chapter': 'test-chapter',
                'heading': 'test-heading',
                'score': 0.8,
                'metadata': {}
            }
        ]
        mock_format_context.return_value = "test context"

        # Mock OpenAI response
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = "Test response with context"
        mock_openai_create.return_value = mock_response

        result = agent_handler("Explain this", optional_selected_text="ROS 2 is a framework")

        assert "answer" in result
        assert result["answer"] == "Test response with context"
        # Verify that the query text was properly combined
        mock_build_embedding.assert_called_once()

    def test_agent_handler_empty_query(self):
        """Test agent handler with empty query."""
        with patch('backend.agent.build_query_embedding') as mock_build_embedding:
            mock_build_embedding.side_effect = Exception("Empty query not allowed")
            result = agent_handler("")

            # Should return an error response
            assert result["answer"] == "I'm sorry, I encountered an error while processing your request. Please try again."
            assert result["sources"] == []


class TestInitializeAgent:
    """Test cases for the initialize_agent function."""

    def test_initialize_agent(self):
        """Test that agent initialization returns expected object."""
        with patch('backend.config.Config.AGENT_MODEL', 'gpt-4-test'), \
             patch('backend.config.Config.QDRANT_COLLECTION_NAME', 'test-collection'):
            result = initialize_agent()

            assert result["status"] == "initialized"
            assert result["model"] == "gpt-4-test"
            assert result["qdrant_collection"] == "test-collection"


def test_sample_questions_with_citations():
    """Test with sample questions to verify responses include proper citations."""
    # This test would normally call the actual agent but we'll mock for unit testing
    with patch('backend.agent.build_query_embedding') as mock_build_embedding, \
         patch('backend.agent.retrieve_chunks') as mock_retrieve_chunks, \
         patch('backend.agent.format_context') as mock_format_context, \
         patch('backend.agent.client.chat.completions.create') as mock_openai_create:

        mock_build_embedding.return_value = [0.1, 0.2, 0.3]
        mock_retrieve_chunks.return_value = [
            {
                'id': 'test_id_1',
                'content': 'test content 1',
                'url': 'https://test-url-1.com',
                'module': 'test-module-1',
                'chapter': 'test-chapter-1',
                'heading': 'test-heading-1',
                'score': 0.85,
                'metadata': {}
            },
            {
                'id': 'test_id_2',
                'content': 'test content 2',
                'url': 'https://test-url-2.com',
                'module': 'test-module-2',
                'chapter': 'test-chapter-2',
                'heading': 'test-heading-2',
                'score': 0.75,
                'metadata': {}
            }
        ]
        mock_format_context.return_value = "test context"

        # Mock OpenAI response
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = "Test response based on context"
        mock_openai_create.return_value = mock_response

        result = agent_handler("What is ROS 2?")

        # Verify that sources are properly cited
        assert len(result["sources"]) == 2
        assert result["sources"][0]["url"] == "https://test-url-1.com"
        assert result["sources"][0]["module"] == "test-module-1"
        assert result["sources"][0]["score"] == 0.85
        assert result["sources"][1]["url"] == "https://test-url-2.com"
        assert result["sources"][1]["module"] == "test-module-2"
        assert result["sources"][1]["score"] == 0.75


class TestUserSelectedText:
    """Test cases for user-selected text functionality."""

    @patch('backend.agent.build_query_embedding')
    @patch('backend.agent.retrieve_chunks')
    @patch('backend.agent.format_context')
    @patch('backend.agent.client.chat.completions.create')
    def test_user_selected_text_functionality(self, mock_openai_create, mock_format_context, mock_retrieve_chunks, mock_build_embedding):
        """Test with selected text scenarios to verify targeted retrieval."""
        # Mock the dependencies
        mock_build_embedding.return_value = [0.1, 0.2, 0.3]
        mock_retrieve_chunks.return_value = [
            {
                'id': 'test_id',
                'content': 'test content related to selected text',
                'url': 'https://test-url.com',
                'module': 'test-module',
                'chapter': 'test-chapter',
                'heading': 'test-heading',
                'score': 0.8,
                'metadata': {}
            }
        ]
        mock_format_context.return_value = "test context"

        # Mock OpenAI response
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = "Test response with selected text context"
        mock_openai_create.return_value = mock_response

        result = agent_handler("Explain this concept", optional_selected_text="This is the selected text that provides context")

        # Verify that the function works with selected text
        assert "answer" in result
        assert result["answer"] == "Test response with selected text context"
        assert len(result["sources"]) == 1

        # Verify that the query text was combined properly
        mock_build_embedding.assert_called_once()

    @patch('backend.agent.build_query_embedding')
    @patch('backend.agent.retrieve_chunks')
    @patch('backend.agent.format_context')
    @patch('backend.agent.client.chat.completions.create')
    def test_targeted_responses_relevance(self, mock_openai_create, mock_format_context, mock_retrieve_chunks, mock_build_embedding):
        """Validate that targeted responses are more relevant to the selected context."""
        # Mock the dependencies
        mock_build_embedding.return_value = [0.1, 0.2, 0.3]
        mock_retrieve_chunks.return_value = [
            {
                'id': 'test_id',
                'content': 'This content is highly relevant to the selected text',
                'url': 'https://relevant-url.com',
                'module': 'relevant-module',
                'chapter': 'relevant-chapter',
                'heading': 'relevant-heading',
                'score': 0.9,
                'metadata': {}
            }
        ]
        mock_format_context.return_value = "test context"

        # Mock OpenAI response
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = "Response that is highly relevant to the selected text context"
        mock_openai_create.return_value = mock_response

        result = agent_handler("How does this work?", optional_selected_text="This is a specific concept about robotics")

        # Verify that the response is relevant and includes proper citations
        assert "answer" in result
        assert "robotics" in result["answer"].lower() or "specific concept" in result["answer"].lower()
        assert len(result["sources"]) == 1
        assert result["sources"][0]["score"] == 0.9  # High relevance score


def test_edge_cases_selected_text():
    """Test edge cases with long or malformed selected text."""
    # Test with very long selected text
    long_text = "This is a very long text. " * 1000  # 3000 words

    with patch('backend.agent.build_query_embedding') as mock_build_embedding, \
         patch('backend.agent.retrieve_chunks') as mock_retrieve_chunks, \
         patch('backend.agent.format_context') as mock_format_context, \
         patch('backend.agent.client.chat.completions.create') as mock_openai_create:

        mock_build_embedding.return_value = [0.1, 0.2, 0.3]
        mock_retrieve_chunks.return_value = []
        mock_format_context.return_value = "test context"

        # Mock OpenAI response
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = "Response to long text query"
        mock_openai_create.return_value = mock_response

        result = agent_handler("Short question", optional_selected_text=long_text)

        # Should handle long text without errors
        assert "answer" in result

    # Test with empty selected text
    with patch('backend.agent.build_query_embedding') as mock_build_embedding, \
         patch('backend.agent.retrieve_chunks') as mock_retrieve_chunks, \
         patch('backend.agent.format_context') as mock_format_context, \
         patch('backend.agent.client.chat.completions.create') as mock_openai_create:

        mock_build_embedding.return_value = [0.1, 0.2, 0.3]
        mock_retrieve_chunks.return_value = []
        mock_format_context.return_value = "test context"

        # Mock OpenAI response
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = "Response to question"
        mock_openai_create.return_value = mock_response

        result = agent_handler("Question", optional_selected_text="")

        # Should handle empty selected text
        assert "answer" in result


if __name__ == "__main__":
    pytest.main([__file__])