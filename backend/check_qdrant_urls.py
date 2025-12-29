import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

load_dotenv()

qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
collection_name = "book_embeddings"

if not qdrant_url or not qdrant_api_key:
    print("Qdrant credentials not found in .env")
else:
    try:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        points, _ = client.scroll(
            collection_name=collection_name,
            limit=100,
            with_payload=True,
            with_vectors=False
        )
        
        urls = set()
        for point in points:
            urls.add(point.payload.get("url"))
            
        print(f"URLs found in Qdrant: {urls}")
        
    except Exception as e:
        print(f"Error checking Qdrant content: {str(e)}")
