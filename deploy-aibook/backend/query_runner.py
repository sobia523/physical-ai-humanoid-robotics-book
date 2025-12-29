"""
Query runner for RAG Pipeline - allows querying the Qdrant collection with user input.

Usage: python query_runner.py --query "your query here"
"""

import argparse
import sys
import logging
from datetime import datetime
import json

# Add backend to path to import modules
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Handle imports based on execution context
try:
    from backend.utils.qdrant_helper import (
        connect_qdrant,
        build_query_embedding,
        run_similarity_search
    )
    from backend.config import Config
except ImportError:
    from utils.qdrant_helper import (
        connect_qdrant,
        build_query_embedding,
        run_similarity_search
    )
    from config import Config


def setup_logging():
    """Set up logging for the query runner."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('query_runner.log'),
            logging.StreamHandler(sys.stdout)
        ]
    )


def run_query(query_text: str, top_k: int = 5):
    """Run a similarity search query against the Qdrant collection."""
    try:
        print(f"\n[QUERY] Query: {query_text}")
        print(f"[INFO] Retrieving top {top_k} similar documents...")
        print("-" * 80)

        # Build query embedding
        query_embedding = build_query_embedding(query_text)
        logging.info(f"Generated embedding for query: {query_text[:50]}...")

        # Run similarity search
        results = run_similarity_search(query_embedding, top_k=top_k)
        logging.info(f"Retrieved {len(results)} results for query: {query_text[:50]}...")

        # Display results
        if results:
            for i, result in enumerate(results, 1):
                print(f"\n{i}. Score: {result['score']:.4f}")
                print(f"   URL: {result['url']}")
                print(f"   Module: {result['module']}")
                print(f"   Chapter: {result['chapter']}")
                print(f"   Heading: {result['heading']}")
                print(f"   Content Preview: {result['chunk_text'][:200]}...")
                print("-" * 80)
        else:
            print("❌ No results found for this query.")

        return results

    except Exception as e:
        logging.error(f"Query failed with error: {str(e)}")
        print(f"\nERROR: Query failed: {str(e)}")
        return []


def main():
    parser = argparse.ArgumentParser(description='RAG Pipeline Query Runner')
    parser.add_argument('--query', '-q', type=str, required=True, help='Query text to search for')
    parser.add_argument('--top-k', type=int, default=5, help='Number of top results to return (default: 5)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Enable verbose logging')

    args = parser.parse_args()

    # Set up logging
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    print("RAG Pipeline Query Runner")
    print("=" * 80)

    try:
        # Validate configuration first
        print("[CONFIG] Validating configuration...")
        Config.validate_config()
        print("SUCCESS: Configuration validation passed")

        # Run the query
        results = run_query(args.query, args.top_k)

        print(f"\nSUCCESS: Query completed. Found {len(results)} results.")
        if results:
            print(f"[SCORE] Top result score: {results[0]['score']:.4f}")

        return 0

    except Exception as e:
        logging.error(f"Query failed with error: {str(e)}")
        print(f"\nERROR: Query failed: {str(e)}")
        return 1


if __name__ == "__main__":
    sys.exit(main())