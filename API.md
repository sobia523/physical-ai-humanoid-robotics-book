# RAG Chatbot API Documentation

This FastAPI application provides HTTP endpoints for the RAG Chatbot Agent that answers questions grounded in book content.

## Endpoints

### GET /
- **Description**: Root endpoint for basic health check
- **Response**: Basic service information

### GET /health
- **Description**: Health check endpoint to verify the service is running
- **Response**:
  ```json
  {
    "status": "healthy",
    "timestamp": "2024-01-01T12:00:00"
  }
  ```

### GET /config
- **Description**: Get configuration information about the agent
- **Response**: Agent configuration details

### POST /ask
- **Description**: Ask a question and get a grounded response from the RAG agent
- **Request Body**:
  ```json
  {
    "prompt": "Your question here",
    "optional_selected_text": "Optional context text (can be null)"
  }
  ```
- **Response**:
  ```json
  {
    "answer": "The answer to your question",
    "sources": ["source1", "source2", ...],
    "confidence": 0.85,
    "timestamp": "2024-01-01T12:00:00"
  }
  ```

### POST /ask-with-context
- **Description**: Ask a question with user-selected text as additional context
- **Request Body**:
  ```json
  {
    "prompt": "Your question here",
    "optional_selected_text": "The selected text to use as context"
  }
  ```
- **Response**: Same as `/ask` endpoint

## Usage

1. Start the server:
   ```bash
   python -m uvicorn backend.app:app --host 0.0.0.0 --port 8000 --reload
   ```

2. Or use the provided batch file on Windows:
   ```bash
   start_api.bat
   ```

3. Test the API using the test script:
   ```bash
   python backend/test_api.py
   ```

## Environment Variables

The API requires the following environment variables to be set in a `.env` file:
- `OPENAI_API_KEY` or `OPENROUTER_API_KEY`: API key for OpenAI or OpenRouter
- `QDRANT_URL`: URL for the Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant
- `QDRANT_COLLECTION_NAME`: Name of the collection (default: book_embeddings)
- `COHERE_API_KEY`: API key for Cohere embeddings
- `SIMILARITY_TOP_K`: Number of top results to return (default: 5)
- `SIMILARITY_THRESHOLD`: Similarity threshold (default: 0.5)

## Dependencies

- fastapi
- uvicorn
- python-dotenv
- agents (OpenAI Agents SDK)
- openai
- pydantic
- qdrant-client
- cohere
- requests