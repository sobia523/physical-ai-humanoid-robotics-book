import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

load_dotenv()

qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
collection_name = "book_embeddings"

target_urls = [
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/01/introduction",
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/02/ros2-fundamentals",
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/03/python-agents-bridging",
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/04/urdf-humanoids",
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/05/practical-integration"
]

if not qdrant_url or not qdrant_api_key:
    print("Qdrant credentials not found in .env")
else:
    try:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        
        for url in target_urls:
            points, _ = client.scroll(
                collection_name=collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="url",
                            match=models.MatchValue(value=url)
                        )
                    ]
                ),
                limit=1
            )
            if points:
                print(f"[FOUND] {url}")
            else:
                print(f"[MISSING] {url}")
                
    except Exception as e:
        print(f"Error checking Qdrant: {str(e)}")
