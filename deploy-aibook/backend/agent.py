"""
RAG Chatbot Agent Implementation using OpenAI Agents SDK

This module implements an OpenAI Agent that integrates with Qdrant-based retrieval
to answer questions grounded in book content. The agent uses a retrieval tool to
query Qdrant for relevant content chunks, then generates responses based solely
on retrieved information with proper source citations.
"""
import os
import sys
import logging
from typing import Dict, List, Optional, Any, Annotated
from datetime import datetime
from pydantic import BaseModel, Field
from agents import Agent, Runner, function_tool
import asyncio
from agents import OpenAIChatCompletionsModel
from openai import AsyncOpenAI
from dotenv import load_dotenv

# Add the project root to the path to allow imports
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, project_root)

# Handle import based on execution context
try:
    from backend.config import Config
    from backend.utils.qdrant_helper import (
        connect_qdrant,
        build_query_embedding,
        run_similarity_search
    )
except ImportError:
    from config import Config
    from utils.qdrant_helper import (
        connect_qdrant,
        build_query_embedding,
        run_similarity_search
    )

load_dotenv()

client = AsyncOpenAI(
    api_key=Config.OPENROUTER_API_KEY,
    base_url="https://openrouter.ai/api/v1",
)

third_party_model = OpenAIChatCompletionsModel(
    openai_client=client,
    model="xiaomi/mimo-v2-flash:free"
)


def format_context(results):
    """
    Format retrieved results into a context string for the agent.

    Args:
        results: List of retrieved chunks with metadata

    Returns:
        Formatted context string
    """
    if not results:
        return "No relevant content found in the knowledge base."

    formatted_context = "Relevant information from the book:\n\n"
    for i, result in enumerate(results, 1):
        formatted_context += f"Source {i}:\n"
        formatted_context += f"URL: {result.get('url', result.get('payload', {}).get('url', ''))}\n"
        formatted_context += f"Module: {result.get('module', result.get('payload', {}).get('module', ''))}\n"
        formatted_context += f"Chapter: {result.get('chapter', result.get('payload', {}).get('chapter', ''))}\n"
        formatted_context += f"Heading: {result.get('heading', result.get('payload', {}).get('heading', ''))}\n"
        formatted_context += f"Content: {result.get('chunk_text', result.get('payload', {}).get('content', ''))}\n"
        formatted_context += f"Relevance Score: {result.get('score', 0):.3f}\n\n"

    return formatted_context


# Define Pydantic model for retrieval results
class RetrievalResult(BaseModel):
    id: str = Field(description="Unique identifier for the chunk")
    url: str = Field(description="Source URL where the content originated")
    module: str = Field(description="Module name in the book structure")
    chapter: str = Field(description="Chapter name in the book structure")
    heading: str = Field(description="Section heading for the content")
    content: str = Field(description="The actual content of the chunk")
    score: float = Field(description="Similarity score between query and chunk")


class RetrievalResults(BaseModel):
    results: List[RetrievalResult] = Field(description="List of retrieved chunks with metadata")

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create the retrieval function as a function tool using the Agents SDK
@function_tool
def retrieve_from_qdrant(query_text: Annotated[str, "The query text to search for in Qdrant"]) -> RetrievalResults:
    """
    Retrieve relevant chunks from Qdrant based on the query text.
    """
    try:
        # Generate embedding for the query
        query_embedding = build_query_embedding(query_text)

        # Retrieve relevant chunks from Qdrant
        retrieved_chunks = run_similarity_search(query_embedding, top_k=Config.SIMILARITY_TOP_K)

        # Format the results
        formatted_results = []
        for chunk in retrieved_chunks:
            formatted_results.append(RetrievalResult(
                id=chunk.get("id", ""),
                url=chunk.get("url", chunk.get("payload", {}).get("url", "")),
                module=chunk.get("module", chunk.get("payload", {}).get("module", "")),
                chapter=chunk.get("chapter", chunk.get("payload", {}).get("chapter", "")),
                heading=chunk.get("heading", chunk.get("payload", {}).get("heading", "")),
                content=chunk.get("chunk_text", chunk.get("payload", {}).get("content", "")),
                score=chunk.get("score", 0.0)
            ))

        return RetrievalResults(results=formatted_results)

    except Exception as e:
        logger.error(f"Error in retrieval function: {str(e)}")
        # Return empty results on error
        return RetrievalResults(results=[])


async def agent_handler_async(prompt: str, optional_selected_text: Optional[str] = None) -> Dict[str, Any]:
    """
    Async handler for the RAG agent that processes user queries and returns grounded responses.

    Args:
        prompt: The user's question or text input
        optional_selected_text: Optional text selected by the user for context

    Returns:
        Dictionary containing the answer and source citations
    """
    try:
        # Combine prompt with selected text if provided
        query_text = prompt
        if optional_selected_text:
            query_text = f"Context: {optional_selected_text}\n\nQuestion: {prompt}"

        # Create an agent with the retrieval tool
        agent = Agent(
            name="RAG Book Assistant",
            instructions="You are a helpful assistant that answers questions based on information retrieved from the book. Use the retrieval tool to get relevant information before answering. Always cite your sources.",
            model=third_party_model,
            tools=[retrieve_from_qdrant]
        )

        # Run the agent asynchronously
        result = await Runner.run(agent, query_text)

        # Extract the answer from the result
        answer = result.final_output if hasattr(result, 'final_output') else str(result)

        # Try to extract source information if available in the result
        sources = []
        confidence = 0.8  # Default confidence

        # For now, we'll return a simple response since the actual sources extraction
        # depends on how the agent processes the tool results
        result_dict = {
            "answer": answer,
            "sources": sources,
            "confidence": confidence,
            "timestamp": datetime.now().isoformat()
        }

        return result_dict

    except Exception as e:
        logger.error(f"Error in agent_handler_async: {str(e)}")
        # Return a safe response in case of errors
        return {
            "answer": "I'm sorry, I encountered an error while processing your request. Please try again.",
            "sources": [],
            "confidence": 0.0,
            "timestamp": datetime.now().isoformat()
        }


def agent_handler(prompt: str, optional_selected_text: Optional[str] = None) -> Dict[str, Any]:
    """
    Synchronous wrapper for backward compatibility.
    Note: This will fail in async contexts. Use agent_handler_async instead.

    Args:
        prompt: The user's question or text input
        optional_selected_text: Optional text selected by the user for context

    Returns:
        Dictionary containing the answer and source citations
    """
    try:
        # Run the agent synchronously
        query_text = prompt
        if optional_selected_text:
            query_text = f"Context: {optional_selected_text}\n\nQuestion: {prompt}"

        agent = Agent(
            name="RAG Book Assistant",
            instructions="You are a helpful assistant that answers questions based on information retrieved from the book. Use the retrieval tool to get relevant information before answering. Always cite your sources.",
            model=third_party_model,
            tools=[retrieve_from_qdrant]
        )

        result = Runner.run_sync(agent, query_text)
        answer = result.final_output if hasattr(result, 'final_output') else str(result)

        return {
            "answer": answer,
            "sources": [],
            "confidence": 0.8,
            "timestamp": datetime.now().isoformat()
        }

    except Exception as e:
        logger.error(f"Error in agent_handler: {str(e)}")
        return {
            "answer": "I'm sorry, I encountered an error while processing your request. Please try again.",
            "sources": [],
            "confidence": 0.0,
            "timestamp": datetime.now().isoformat()
        }


def initialize_agent():
    """
    Initialize the OpenAI agent with the retrieval tool.
    """
    # Check if required configuration is available
    if not Config.OPENROUTER_API_KEY:
        return {
            "status": "error",
            "message": "OpenRouter API key is not configured.",
            "model": Config.AGENT_MODEL,
            "qdrant_collection": Config.QDRANT_COLLECTION_NAME
        }

    # The agent is initialized when this module is imported
    # We can return a simple confirmation
    return {
        "status": "initialized",
        "model": Config.AGENT_MODEL,
        "qdrant_collection": Config.QDRANT_COLLECTION_NAME
    }


def create_retrieval_tool():
    """
    Create a retrieval tool function that can be used by the OpenAI agent.
    """
    # The retrieval tool is already defined as the retrieve_from_qdrant function tool
    return retrieve_from_qdrant


if __name__ == "__main__":
    # This will be used for testing once implementation is complete
    print("Agent initialized")
    result = agent_handler("What is a Digital Twin?")
    print(result)