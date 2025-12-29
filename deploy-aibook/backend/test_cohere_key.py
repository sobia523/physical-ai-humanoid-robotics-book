import os
import cohere
from dotenv import load_dotenv

load_dotenv()

cohere_api_key = os.getenv("COHERE_API_KEY")

if not cohere_api_key:
    print("COHERE_API_KEY not found in .env")
else:
    try:
        cohere_client = cohere.Client(cohere_api_key)
        response = cohere_client.embed(
            texts=["This is a test."],
            model="embed-english-v3.0",
            input_type="search_document"
        )
        print("Cohere API key is working!")
        print(f"Embedding dimension: {len(response.embeddings[0])}")
    except Exception as e:
        print(f"Error testing Cohere API key: {str(e)}")
