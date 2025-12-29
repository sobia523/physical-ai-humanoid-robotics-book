import os
import sys
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    https=True
)

collection_name = "book_embeddings"

print("Checking Qdrant collection status...")
print("=" * 60)

try:
    # Get collection info
    collection_info = qdrant_client.get_collection(collection_name)
    print(f"Collection '{collection_name}' exists")
    print(f"Points count: {collection_info.points_count}")
    print(f"Vector size: {collection_info.config.params.vectors.size}")
    print(f"Distance: {collection_info.config.params.vectors.distance}")

    # Try to get some points to see what's actually there
    print("\nTrying to get some points from the collection...")
    try:
        # Get first few points using scroll
        points, next_page = qdrant_client.scroll(
            collection_name=collection_name,
            limit=5
        )

        print(f"Retrieved {len(points)} points using scroll:")
        for i, point in enumerate(points):
            print(f"  Point {i+1}: ID={point.id}, Payload keys={list(point.payload.keys()) if point.payload else 'None'}")
            if point.payload:
                print(f"    URL: {point.payload.get('url', 'N/A')}")
    except Exception as e:
        print(f"Error retrieving points with scroll: {e}")

    # Try to count points
    try:
        count = qdrant_client.count(
            collection_name=collection_name,
            exact=True
        )
        print(f"\nExact count: {count.count}")
    except Exception as e:
        print(f"Error counting points: {e}")

except Exception as e:
    print(f"Error accessing Qdrant: {str(e)}")
    import traceback
    traceback.print_exc()