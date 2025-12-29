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

print("Checking Qdrant collection for recent changes...")
print("=" * 60)

try:
    # Get first few points to see the structure
    scroll_result = qdrant_client.scroll(
        collection_name=collection_name,
        limit=10,  # Get first 10 points
        with_payload=True
    )

    print("First 10 points in collection:")
    for i, point in enumerate(scroll_result[0]):
        payload = point.payload
        url = payload.get('url', 'N/A')
        module = payload.get('module', 'N/A')
        chapter = payload.get('chapter', 'N/A')
        print(f"{i+1}. URL: {url}")
        print(f"   Module: {module}")
        print(f"   Chapter: {chapter}")
        print(f"   Point ID: {point.id}")
        print()

    # Count total points
    count = qdrant_client.count(
        collection_name=collection_name,
        exact=True
    )
    print(f"Total points in collection: {count.count}")

    # Check if there are any ROS2 URLs at all
    ros2_points = qdrant_client.scroll(
        collection_name=collection_name,
        scroll_filter=models.Filter(
            must=[
                models.FieldCondition(
                    key="url",
                    match=models.Match(value=models.MatchText(text="ros2"))
                )
            ]
        ),
        limit=100,
        with_payload=True
    )

    print(f"ROS2-related points found: {len(ros2_points[0])}")
    for point in ros2_points[0]:
        payload = point.payload
        print(f"- {payload.get('url', 'N/A')} (ID: {point.id})")

except Exception as e:
    print(f"Error accessing Qdrant: {str(e)}")