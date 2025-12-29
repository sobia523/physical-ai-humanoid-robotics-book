"""
FastAPI Backend for RAG Chatbot Agent

This module creates a FastAPI application that integrates with the RAG chatbot agent
to provide HTTP endpoints for question answering based on book content.
"""
import os
import logging
from typing import Optional, Dict, Any
from datetime import datetime
from fastapi import FastAPI, HTTPException, Request
from pydantic import BaseModel
from dotenv import load_dotenv

# Add the project root to the path to allow imports
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, project_root)

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import the agent handler
try:
    from backend.agent import agent_handler, initialize_agent
except ImportError as e:
    logger.error(f"Failed to import agent module: {e}")
    from agent import agent_handler, initialize_agent

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG Chatbot that answers questions based on book content",
    version="1.0.0"
)

# Define request/response models
class QuestionRequest(BaseModel):
    prompt: str
    optional_selected_text: Optional[str] = None

class AnswerResponse(BaseModel):
    answer: str
    sources: list
    confidence: float
    timestamp: str

class HealthCheck(BaseModel):
    status: str
    timestamp: str

@app.get("/")
async def root():
    """Root endpoint for basic health check."""
    return {"message": "RAG Chatbot API is running", "timestamp": datetime.now().isoformat()}

@app.get("/health", response_model=HealthCheck)
async def health_check():
    """Health check endpoint to verify the service is running."""
    try:
        # Test agent initialization
        init_result = initialize_agent()
        if init_result.get("status") == "error":
            raise HTTPException(status_code=500, detail=init_result.get("message", "Agent initialization failed"))

        return HealthCheck(
            status="healthy",
            timestamp=datetime.now().isoformat()
        )
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Health check failed: {str(e)}")

@app.post("/ask", response_model=AnswerResponse)
async def ask_question(request: QuestionRequest):
    """
    Endpoint to ask a question and get a grounded response from the RAG agent.

    Args:
        request: QuestionRequest containing the prompt and optional selected text

    Returns:
        AnswerResponse containing the answer, sources, confidence, and timestamp
    """
    try:
        logger.info(f"Received question: {request.prompt[:100]}...")

        # Call the agent handler with the prompt and optional selected text
        result = agent_handler(
            prompt=request.prompt,
            optional_selected_text=request.optional_selected_text
        )

        # Validate the result structure
        if not isinstance(result, dict):
            raise HTTPException(status_code=500, detail="Invalid response format from agent")

        # Ensure required fields are present
        required_fields = ["answer", "sources", "confidence", "timestamp"]
        for field in required_fields:
            if field not in result:
                raise HTTPException(status_code=500, detail=f"Missing required field: {field}")

        return AnswerResponse(**result)

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error processing question: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing question: {str(e)}"
        )

@app.post("/ask-with-context", response_model=AnswerResponse)
async def ask_with_context(request: QuestionRequest):
    """
    Endpoint to ask a question with user-selected text as additional context.
    This endpoint specifically handles cases where the user provides selected text
    that should be used as additional context for the answer.

    Args:
        request: QuestionRequest containing the prompt and selected text

    Returns:
        AnswerResponse containing the answer, sources, confidence, and timestamp
    """
    try:
        logger.info(f"Received question with context: {request.prompt[:100]}...")

        if not request.optional_selected_text:
            raise HTTPException(
                status_code=400,
                detail="optional_selected_text is required for this endpoint"
            )

        # Call the agent handler with both prompt and selected text
        result = agent_handler(
            prompt=request.prompt,
            optional_selected_text=request.optional_selected_text
        )

        # Validate the result structure
        if not isinstance(result, dict):
            raise HTTPException(status_code=500, detail="Invalid response format from agent")

        # Ensure required fields are present
        required_fields = ["answer", "sources", "confidence", "timestamp"]
        for field in required_fields:
            if field not in result:
                raise HTTPException(status_code=500, detail=f"Missing required field: {field}")

        return AnswerResponse(**result)

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error processing question with context: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing question with context: {str(e)}"
        )

@app.get("/config")
async def get_config():
    """
    Endpoint to get configuration information about the agent.
    """
    try:
        init_result = initialize_agent()
        return {
            "agent_config": init_result,
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        logger.error(f"Error getting config: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error getting config: {str(e)}")

# Error handlers
@app.exception_handler(404)
async def not_found_handler(request: Request, exc: HTTPException):
    return {"error": "Endpoint not found", "path": str(request.url)}

@app.exception_handler(500)
async def internal_error_handler(request: Request, exc: HTTPException):
    logger.error(f"Internal error at {request.url}: {str(exc)}")
    return {"error": "Internal server error", "detail": str(exc)}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=True
    )