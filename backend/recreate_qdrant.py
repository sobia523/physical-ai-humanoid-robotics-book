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

print("Recreating Qdrant collection to start fresh...")
print("=" * 60)

try:
    # Get current collection info
    try:
        collection_info = qdrant_client.get_collection(collection_name)
        print(f"Collection '{collection_name}' exists with {collection_info.points_count} points")
    except:
        print(f"Collection '{collection_name}' does not exist")

    # Delete the existing collection
    print(f"Deleting collection '{collection_name}'...")
    qdrant_client.delete_collection(collection_name)
    print(f"Collection '{collection_name}' deleted successfully")

    # Recreate the collection with proper configuration
    from qdrant_client.http import models

    qdrant_client.create_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),  # Cohere embeddings are 1024-dimensional
    )

    print(f"Collection '{collection_name}' recreated successfully")

    # Create indexes for efficient searching
    try:
        qdrant_client.create_payload_index(
            collection_name=collection_name,
            field_name="url",
            field_schema=models.PayloadSchemaType.KEYWORD
        )
        print("Created index for 'url' field")
    except Exception as e:
        print(f"Could not create index for 'url' field: {e}")

    try:
        qdrant_client.create_payload_index(
            collection_name=collection_name,
            field_name="module",
            field_schema=models.PayloadSchemaType.KEYWORD
        )
        print("Created index for 'module' field")
    except Exception as e:
        print(f"Could not create index for 'module' field: {e}")

    try:
        qdrant_client.create_payload_index(
            collection_name=collection_name,
            field_name="chapter",
            field_schema=models.PayloadSchemaType.KEYWORD
        )
        print("Created index for 'chapter' field")
    except Exception as e:
        print(f"Could not create index for 'chapter' field: {e}")

    # Check that the collection is now empty
    count = qdrant_client.count(
        collection_name=collection_name,
        exact=True
    )
    print(f"New collection has {count.count} points (should be 0)")

    print("Qdrant collection reset successfully!")

except Exception as e:
    print(f"Error during Qdrant collection reset: {str(e)}")
    import traceback
    traceback.print_exc()