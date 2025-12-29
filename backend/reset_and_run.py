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

print("Deleting collection to ensure fresh start...")
try:
    qdrant_client.delete_collection(collection_name)
    print("Collection deleted successfully")
except Exception as e:
    print(f"Collection may not have existed: {e}")

# Recreate the collection
qdrant_client.create_collection(
    collection_name=collection_name,
    vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
)

# Create indexes
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

# Check that collection is empty
count = qdrant_client.count(
    collection_name=collection_name,
    exact=True
)
print(f"New collection has {count.count} points (should be 0)")

print("Collection reset complete. Now running pipeline...")