import os
import sys
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Load environment variables
load_dotenv()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    https=True
)

collection_name = "book_embeddings"

# Specific ROS2 URLs to verify
ros2_urls = [
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/01/introduction",
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/02/ros2-fundamentals",
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/03/python-agents-bridging",
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/04/urdf-humanoids",
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/05/practical-integration"
]

print("Checking for specific ROS2 nervous system chapter URLs in Qdrant...")
print("=" * 60)

found_urls = []
missing_urls = []

for url in ros2_urls:
    try:
        # Search for points with this URL in the payload
        search_result = qdrant_client.scroll(
            collection_name=collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="url",
                        match=models.MatchValue(value=url)
                    )
                ]
            ),
            limit=1  # We just need to know if at least one exists
        )

        if search_result[0]:  # If we found at least one point with this URL
            found_urls.append(url)
            print(f"[FOUND] Found: {url}")

            # Get more details about the first found entry
            sample_point = search_result[0][0]
            payload = sample_point.payload
            print(f"  - Module: {payload.get('module', 'N/A')}")
            print(f"  - Chapter: {payload.get('chapter', 'N/A')}")
            print(f"  - Content length: {len(payload.get('content', ''))} characters")
            print(f"  - Point ID: {sample_point.id}")
        else:
            missing_urls.append(url)
            print(f"[MISSING] Missing: {url}")

    except Exception as e:
        print(f"[ERROR] Error checking {url}: {str(e)}")
        missing_urls.append(url)

print("=" * 60)
print(f"Summary: {len(found_urls)} out of {len(ros2_urls)} ROS2 URLs found in Qdrant")
print(f"Found: {len(found_urls)}")
print(f"Missing: {len(missing_urls)}")

if missing_urls:
    print("\nMissing URLs:")
    for url in missing_urls:
        print(f"  - {url}")
else:
    print("\nAll ROS2 nervous system chapter URLs are present in Qdrant!")

# Also check total count of documents in the collection
try:
    count = qdrant_client.count(
        collection_name=collection_name,
        exact=True
    )
    print(f"\nTotal documents in collection: {count.count}")
except Exception as e:
    print(f"Error getting total count: {str(e)}")