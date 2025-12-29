"""
Connection validation script for RAG Pipeline Retrieval Validation.

This script validates the connection to the Qdrant collection and performs basic checks
to ensure the validation system can access the stored embeddings and metadata.
"""

import logging
import sys
from datetime import datetime

# Add the backend directory to the path for imports
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from utils.qdrant_helper import connect_qdrant, inspect_collection, fetch_sample_records
from utils.validation import generate_validation_report, save_validation_report
from config import Config


def setup_logging():
    """Set up logging for the validation script."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('validation_connection.log'),
            logging.StreamHandler(sys.stdout)
        ]
    )


def test_connection():
    """
    Test connection to Qdrant and perform basic validation checks.

    Returns:
        bool: True if connection and basic tests pass, False otherwise
    """
    try:
        logging.info("Starting connection test to Qdrant...")

        # Test basic connection
        client = connect_qdrant()
        logging.info("SUCCESS: Successfully connected to Qdrant")

        # Inspect collection
        collection_info = inspect_collection()
        logging.info(f"SUCCESS: Collection '{collection_info['collection_name']}' found with {collection_info['vector_count']} vectors")
        logging.info(f"SUCCESS: Vector dimensions: {collection_info['vector_config']['size']}")
        logging.info(f"SUCCESS: Distance metric: {collection_info['vector_config']['distance']}")

        # Fetch sample records
        sample_records = fetch_sample_records(limit=Config.SAMPLE_RECORD_LIMIT)
        logging.info(f"SUCCESS: Successfully retrieved {len(sample_records)} sample records")

        # Validate that records have required fields
        if sample_records:
            # Check the payload structure - fields are stored in the payload
            required_fields = ['url', 'content', 'module']  # Using actual field names from Qdrant
            first_record = sample_records[0]
            payload = first_record.get('payload', {})

            for field in required_fields:
                if field in payload and payload[field] not in [None, '', 'N/A', 'None']:
                    logging.info(f"SUCCESS: Required field '{field}' present in sample record")
                else:
                    logging.warning(f"WARNING: Required field '{field}' missing in sample record")
                    return False
        else:
            logging.error("ERROR: No sample records found in collection")
            return False

        logging.info("SUCCESS: All connection tests passed successfully")
        return True

    except Exception as e:
        logging.error(f"ERROR: Connection test failed: {str(e)}")
        return False


def main():
    """Main function to run the connection validation."""
    setup_logging()

    print("=" * 60)
    print("RAG Pipeline Retrieval Validation - Connection Test")
    print("=" * 60)

    # Validate configuration first
    try:
        Config.validate_config()
        logging.info("SUCCESS: Configuration validation passed")
    except ValueError as e:
        logging.error(f"ERROR: Configuration validation failed: {str(e)}")
        print(f"Configuration error: {str(e)}")
        return False

    # Run connection tests
    success = test_connection()

    print("\n" + "=" * 60)
    if success:
        print("CONNECTION VALIDATION: PASSED")
        print("The validation system can successfully connect to Qdrant")
        print("and access stored embeddings and metadata.")
    else:
        print("CONNECTION VALIDATION: FAILED")
        print("The validation system could not connect to Qdrant or")
        print("failed basic validation checks.")
    print("=" * 60)

    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)