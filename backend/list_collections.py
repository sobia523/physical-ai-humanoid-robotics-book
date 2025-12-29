import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if not qdrant_url or not qdrant_api_key:
    print("Qdrant credentials not found in .env")
else:
    try:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        collections = client.get_collections()
        for collection in collections.collections:
            print(f"Collection: {collection.name}")
            info = client.get_collection(collection.name)
            print(f"  Vectors count: {info.points_count}")
    except Exception as e:
        print(f"Error listing collections: {str(e)}")