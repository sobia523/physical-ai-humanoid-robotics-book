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

print("Checking for all URLs containing 'ros2' in Qdrant...")
print("=" * 60)

try:
    # Get all points from the collection with URL field
    scroll_result = qdrant_client.scroll(
        collection_name=collection_name,
        limit=1000,  # Get up to 1000 points
        with_payload=True
    )

    ros2_urls = []
    all_urls = []

    for point in scroll_result[0]:
        payload = point.payload
        url = payload.get('url', '')
        all_urls.append(url)
        if 'ros2' in url.lower():
            ros2_urls.append(url)
            print(f"[FOUND] {url}")
            print(f"  - Module: {payload.get('module', 'N/A')}")
            print(f"  - Chapter: {payload.get('chapter', 'N/A')}")
            print(f"  - Point ID: {point.id}")
            print(f"  - Content length: {len(payload.get('content', ''))} chars")
            print()

    print(f"Total URLs in collection: {len(all_urls)}")
    print(f"ROS2-related URLs found: {len(ros2_urls)}")

    if ros2_urls:
        print("\nAll ROS2 URLs found:")
        for url in ros2_urls:
            print(f"  - {url}")
    else:
        print("\nNo ROS2 URLs found in the collection!")
        print("Let's check if any new data was added by comparing total count...")

        # Check total count
        count = qdrant_client.count(
            collection_name=collection_name,
            exact=True
        )
        print(f"Total documents in collection: {count.count}")

        # Check for any new data by looking at recent additions
        print("\nChecking for any recently processed URLs...")
        recent_urls = [url for url in all_urls if 'docs/modules/' in url]
        print(f"Found {len(recent_urls)} documentation URLs in collection")
        if recent_urls:
            print("Sample of documentation URLs:")
            for url in recent_urls[:10]:  # Show first 10
                print(f"  - {url}")

except Exception as e:
    print(f"[ERROR] Error accessing Qdrant: {str(e)}")