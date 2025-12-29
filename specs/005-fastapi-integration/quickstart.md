# Quickstart: RAG Chatbot FastAPI Backend Integration

**Feature**: RAG Chatbot — FastAPI Backend Integration
**Date**: 2025-12-29
**Branch**: 005-fastapi-integration

## Overview

This quickstart guide provides instructions for setting up, running, and testing the FastAPI backend service that integrates with the RAG chatbot agent.

## Prerequisites

- Python 3.11+
- pip package manager
- Access to OpenAI API key (or OpenRouter API key)
- Access to Qdrant vector database credentials
- Access to Cohere API key

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd Physical-AI-Humanoid-Robotics-Textbook
```

### 2. Navigate to Backend Directory
```bash
cd backend
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

### 4. Set Up Environment Variables
Create a `.env` file in the project root with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key
OPENROUTER_API_KEY=your_openrouter_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_embeddings
COHERE_API_KEY=your_cohere_api_key
SIMILARITY_TOP_K=5
SIMILARITY_THRESHOLD=0.5
```

## Running the API Service

### 1. Start the FastAPI Server
```bash
# From the project root directory
uvicorn backend.api:app --host 0.0.0.0 --port 8000 --reload
```

### 2. Alternative: Use the Startup Script
```bash
# On Windows
./start_api.bat

# On Linux/Mac
./start_api.sh  # (if created)
```

## API Endpoints

### Health Check
- **GET** `/health`
- Check if the service is running and all dependencies are available

### Ask Endpoint
- **POST** `/ask`
- Ask a general question about the book content
- Request body:
```json
{
  "query": "Your question here"
}
```

### Ask with Selected Text Endpoint
- **POST** `/ask-selected`
- Ask a question with additional context from selected text
- Request body:
```json
{
  "query": "Your question here",
  "selected_text": "Additional context text here"
}
```

## Testing the API

### 1. Test Health Check
```bash
curl http://localhost:8000/health
```

### 2. Test Ask Endpoint
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"query": "What is a Digital Twin?"}'
```

### 3. Test Ask with Selected Text Endpoint
```bash
curl -X POST http://localhost:8000/ask-selected \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept",
    "selected_text": "Digital twins are virtual replicas of physical systems..."
  }'
```

## Configuration

### Environment Variables
- `OPENAI_API_KEY`: API key for OpenAI service
- `OPENROUTER_API_KEY`: API key for OpenRouter service (alternative to OpenAI)
- `QDRANT_URL`: URL for the Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant
- `QDRANT_COLLECTION_NAME`: Name of the collection in Qdrant (default: book_embeddings)
- `COHERE_API_KEY`: API key for Cohere embeddings
- `SIMILARITY_TOP_K`: Number of top results to return (default: 5)
- `SIMILARITY_THRESHOLD`: Minimum similarity threshold (default: 0.5)

### Port Configuration
The API runs on port 8000 by default. To change this, modify the uvicorn command:
```bash
uvicorn backend.api:app --host 0.0.0.0 --port 8080
```

## Troubleshooting

### Common Issues

1. **API Keys Not Set**
   - Ensure all required environment variables are set
   - Check that the `.env` file is properly loaded

2. **Qdrant Connection Issues**
   - Verify QDRANT_URL and QDRANT_API_KEY are correct
   - Ensure the Qdrant service is running and accessible

3. **Cohere Connection Issues**
   - Verify COHERE_API_KEY is correct
   - Check Cohere service availability

4. **Port Already in Use**
   - Change the port number in the uvicorn command
   - Ensure no other processes are using port 8000

### Logs
Check the console output for detailed error messages and logs during development.