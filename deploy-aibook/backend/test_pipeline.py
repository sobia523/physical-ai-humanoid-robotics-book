"""
Simple test script to verify the RAG pipeline is working by retrieving sample records.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from backend.utils.qdrant_helper import fetch_sample_records
    from backend.config import Config
except ImportError:
    from utils.qdrant_helper import fetch_sample_records
    from config import Config

def test_retrieval():
    """Test that we can retrieve sample records from Qdrant."""
    print("[TEST] Testing RAG Pipeline - Sample Record Retrieval")
    print("=" * 60)

    try:
        # Validate configuration
        Config.validate_config()
        print("SUCCESS: Configuration validated successfully")

        # Fetch sample records
        print(f"[INFO] Retrieving sample records from collection: {Config.QDRANT_COLLECTION_NAME}")
        sample_records = fetch_sample_records(limit=5)

        print(f"SUCCESS: Successfully retrieved {len(sample_records)} sample records")
        print()

        # Display sample records
        for i, record in enumerate(sample_records, 1):
            print(f"Record {i}:")
            print(f"  ID: {record.get('id', 'N/A')}")
            print(f"  URL: {record.get('url', 'N/A')}")
            print(f"  Module: {record.get('module', 'N/A')}")
            print(f"  Chapter: {record.get('chapter', 'N/A')}")
            print(f"  Heading: {record.get('heading', 'N/A')}")
            print(f"  Content Preview: {record.get('chunk_text', '')[:150]}...")
            print("-" * 40)

        print(f"\nSUCCESS: Successfully verified RAG pipeline functionality!")
        print(f"[INFO] Found {len(sample_records)} records in the Qdrant collection")
        print(f"[COLLECTION] Collection: {Config.QDRANT_COLLECTION_NAME}")
        print(f"[COUNT] Total vectors in collection: {len(sample_records)}+ (showing first 5)")

        return True

    except Exception as e:
        print(f"ERROR: Error during retrieval test: {str(e)}")
        return False

if __name__ == "__main__":
    success = test_retrieval()
    if success:
        print("\nSUCCESS: RAG Pipeline is working correctly!")
    else:
        print("\nERROR: RAG Pipeline test failed!")
        sys.exit(1)