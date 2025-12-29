# Quickstart Guide: RAG Chatbot — Agent + Retrieval Implementation

## Overview
This guide provides instructions for setting up and using the RAG Chatbot Agent that integrates with Qdrant-based retrieval to answer questions grounded in book content.

## Prerequisites
- Python 3.11+
- OpenAI API key
- Qdrant Cloud account and collection with book embeddings
- Cohere API key (for embedding generation)
- Environment variables configured (see Configuration section)

## Installation

1. **Clone the repository**
```bash
git clone <repository-url>
cd <repository-name>
```

2. **Install dependencies**
```bash
pip install openai qdrant-client cohere python-dotenv requests
```

3. **Configure environment variables** (see Configuration section)

## Configuration

Create a `.env` file in the backend directory with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=your_collection_name
COHERE_API_KEY=your_cohere_api_key
COHERE_MODEL=embed-multilingual-v2.0  # or your embedding model
SIMILARITY_TOP_K=5
SIMILARITY_THRESHOLD=0.5
```

## Usage

### 1. Initialize the Agent
```python
from backend.agent import agent_handler

# Initialize and use the agent
response = agent_handler("What is ROS 2?", selected_text=None)
print(response)
```

### 2. Query with Selected Text
```python
selected_text = "Robot Operating System 2 (ROS 2) is a flexible framework for writing robot applications."
response = agent_handler("Explain the benefits of this system", selected_text=selected_text)
print(response)
```

### 3. Using the Agent Directly
```python
from backend.agent import initialize_agent

agent = initialize_agent()
response = agent.query("How does humanoid robot motion planning work?")
print(response)
```

## API Contract

### Request Format
```json
{
  "query": "Your question here",
  "selected_text": "Optional selected text for context (can be null)"
}
```

### Response Format
```json
{
  "answer": "The agent's response to your query",
  "sources": [
    {
      "url": "https://source-url",
      "module": "module-name",
      "chapter": "chapter-name",
      "heading": "section-heading",
      "score": 0.85
    }
  ],
  "confidence": 0.92
}
```

## Testing

### Run Unit Tests
```bash
cd backend
python -m pytest tests/test_agent.py -v
```

### Test Retrieval Functionality
```bash
cd backend
python -m pytest tests/test_retrieval.py -v
```

### Manual Testing
```python
# Test basic functionality
from backend.agent import agent_handler

# Test 1: Basic question
result1 = agent_handler("What is ROS 2?")
print("Test 1 - Basic question:", result1)

# Test 2: Question with selected text
result2 = agent_handler("What are the benefits?", selected_text="ROS 2 is a flexible framework for writing robot applications.")
print("Test 2 - With selected text:", result2)

# Test 3: Complex question
result3 = agent_handler("How does motion planning work in humanoid robots?")
print("Test 3 - Complex question:", result3)
```

## Troubleshooting

### Common Issues

1. **API Keys Not Found**
   - Ensure all required API keys are set in the `.env` file
   - Verify the environment file is loaded properly

2. **Qdrant Connection Issues**
   - Check QDRANT_URL and QDRANT_API_KEY are correct
   - Verify the collection exists and contains embeddings

3. **Low-Quality Responses**
   - Adjust SIMILARITY_THRESHOLD if getting too many low-quality results
   - Check that the embedding model matches the one used for indexing

4. **Rate Limits**
   - If hitting API rate limits, implement request throttling
   - Consider upgrading API plan if needed