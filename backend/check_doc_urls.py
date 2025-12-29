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

print("Checking for all documentation URLs in Qdrant...")
print("=" * 60)

try:
    # Get all points from the collection with URL field
    scroll_result = qdrant_client.scroll(
        collection_name=collection_name,
        limit=1000,  # Get up to 1000 points
        with_payload=True
    )

    doc_urls = []
    ros2_chapter_urls = []

    for point in scroll_result[0]:
        payload = point.payload
        url = payload.get('url', '')

        # Check for documentation URLs
        if 'docs/modules/' in url:
            doc_urls.append(url)

            # Check for the specific ROS2 chapter URLs
            if '/ros2-nervous-system/chapters/' in url:
                ros2_chapter_urls.append(url)
                print(f"[FOUND] {url}")
                print(f"  - Module: {payload.get('module', 'N/A')}")
                print(f"  - Chapter: {payload.get('chapter', 'N/A')}")
                print(f"  - Point ID: {point.id}")
                print(f"  - Content length: {len(payload.get('content', ''))} chars")
                print()

    print(f"Total documentation URLs found: {len(doc_urls)}")
    print(f"Specific ROS2 chapter URLs found: {len(ros2_chapter_urls)}")

    if ros2_chapter_urls:
        print("\nAll ROS2 chapter URLs found:")
        for url in ros2_chapter_urls:
            print(f"  - {url}")
    else:
        print("\nNo specific ROS2 chapter URLs found!")

        # Show some sample documentation URLs to confirm the collection was updated
        print(f"\nSample of documentation URLs in collection ({len(doc_urls)} total):")
        for url in doc_urls[:15]:  # Show first 15
            print(f"  - {url}")

    # Check total count
    count = qdrant_client.count(
        collection_name=collection_name,
        exact=True
    )
    print(f"\nTotal documents in collection: {count.count}")

except Exception as e:
    print(f"[ERROR] Error accessing Qdrant: {str(e)}")