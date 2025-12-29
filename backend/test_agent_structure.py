"""
Simple test to verify the agent structure without calling external APIs.
"""
import os
from unittest.mock import patch, MagicMock

# Temporarily set environment variables for testing
os.environ['OPENAI_API_KEY'] = 'test-key'
os.environ['QDRANT_URL'] = 'https://test.qdrant.com'
os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'
os.environ['COHERE_API_KEY'] = 'test-cohere-key'

# Import after setting environment variables
from backend.agent import agent_handler, initialize_agent

def test_agent_structure():
    """Test that the agent structure is correct."""
    print("Testing agent structure...")

    # Test initialization
    init_result = initialize_agent()
    print(f"Initialization result: {init_result}")

    # Test that functions exist
    assert callable(agent_handler), "agent_handler should be callable"
    assert callable(initialize_agent), "initialize_agent should be callable"

    print("✓ Agent structure is correct")

    # Test that agent_handler handles errors gracefully
    with patch('backend.agent.build_query_embedding') as mock_embedding, \
         patch('backend.agent.run_similarity_search') as mock_search, \
         patch('backend.agent.format_context') as mock_format, \
         patch('backend.agent.client.chat.completions.create') as mock_openai:

        # Simulate an error to test error handling
        mock_embedding.side_effect = Exception("Test error")

        result = agent_handler("What is ROS 2?")
        print(f"Error handling result: {result}")

        assert result["answer"] == "I'm sorry, I encountered an error while processing your request. Please try again."
        assert result["sources"] == []

        print("✓ Error handling works correctly")

if __name__ == "__main__":
    test_agent_structure()
    print("\n✓ All agent structure tests passed!")