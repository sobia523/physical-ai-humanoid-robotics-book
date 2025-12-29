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

print("Checking for all ROS2 content in Qdrant collection...")
print("=" * 60)

try:
    # Get all points from the collection
    all_points = []
    offset = None

    while True:
        batch, next_offset = qdrant_client.scroll(
            collection_name=collection_name,
            limit=100,
            offset=offset,
            with_payload=True
        )

        all_points.extend(batch)

        if next_offset is None:
            break
        offset = next_offset

    print(f"Retrieved {len(all_points)} total points from collection")

    # Extract unique URLs
    unique_urls = set()
    ros2_urls = []

    for point in all_points:
        if point.payload:
            url = point.payload.get('url', '')
            if 'ros2-nervous-system' in url:
                ros2_urls.append(url)
                unique_urls.add(url)

    print(f"Found {len(ros2_urls)} ROS2-related entries")
    print(f"Found {len(unique_urls)} unique ROS2 URLs:")

    for url in sorted(unique_urls):
        count = ros2_urls.count(url)
        print(f"  - {url} ({count} chunks)")

    # Specifically check for the target URLs
    target_urls = [
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/01/introduction",
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/02/ros2-fundamentals",
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/03/python-agents-bridging",
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/04/urdf-humanoids",
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/05/practical-integration"
    ]

    print("\nChecking for target ROS2 chapter URLs:")
    for target_url in target_urls:
        found_count = ros2_urls.count(target_url)
        status = "FOUND" if found_count > 0 else "MISSING"
        print(f"  {status}: {target_url} ({found_count} chunks)")

except Exception as e:
    print(f"Error accessing Qdrant: {str(e)}")
    import traceback
    traceback.print_exc()