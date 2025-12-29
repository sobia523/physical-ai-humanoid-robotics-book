"""
Qdrant helper module for RAG Pipeline Retrieval Validation.

This module provides utility functions for connecting to Qdrant, retrieving sample records,
building query embeddings, running similarity searches, and validating results.
"""

from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, Filter, FieldCondition, MatchValue
from sentence_transformers import SentenceTransformer

# Handle import based on execution context
try:
    from backend.config import Config
except ImportError:
    from config import Config

import logging
import time
from typing import List, Dict, Any, Optional


def connect_qdrant() -> QdrantClient:
    """
    Establish connection to Qdrant Cloud instance using configuration values.

    Returns:
        QdrantClient: Connected Qdrant client instance
    """
    try:
        client = QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
            timeout=30,  # 30 second timeout
            https=True,
            prefer_grpc=True
        )

        # Test connection by getting collections
        collections = client.get_collections()
        logging.info(f"Successfully connected to Qdrant. Found {len(collections.collections)} collections")

        return client
    except Exception as e:
        logging.error(f"Failed to connect to Qdrant: {str(e)}")
        raise


def fetch_sample_records(limit: int = 10) -> List[Dict[str, Any]]:
    """
    Fetch sample records from the Qdrant collection with metadata.

    Args:
        limit: Number of sample records to retrieve (default 10)

    Returns:
        List of records with metadata including URL, chunk text, and other fields
    """
    try:
        client = connect_qdrant()

        # Retrieve points with payload (metadata) and vectors
        records = client.scroll(
            collection_name=Config.QDRANT_COLLECTION_NAME,
            limit=limit,
            with_payload=True,
            with_vectors=False
        )

        sample_records = []
        for record in records[0]:  # records is a tuple (records, next_page_offset)
            payload = record.payload or {}
            sample_records.append({
                'id': record.id,
                'payload': payload,
                'url': payload.get('url', 'N/A'),
                'chunk_text': payload.get('content', payload.get('chunk_text', 'N/A')),
                'module': payload.get('module', 'N/A'),
                'chapter': payload.get('chapter', 'N/A'),
                'heading': payload.get('section_title', payload.get('heading', 'N/A')),
                'timestamp': payload.get('timestamp', 'N/A')
            })

        logging.info(f"Retrieved {len(sample_records)} sample records from Qdrant collection")
        return sample_records

    except Exception as e:
        logging.error(f"Failed to fetch sample records: {str(e)}")
        raise


# Global model cache to avoid reloading for every query
_model_cache = None

def build_query_embedding(text: str) -> List[float]:
    """
    Build embedding vector for query text using a local SentenceTransformer model.

    Args:
        text: Input text to convert to embedding

    Returns:
        List of float values representing the embedding vector
    """
    global _model_cache
    try:
        if _model_cache is None:
            model_name = "all-MiniLM-L6-v2"  # Matches the local ingestion pipeline
            _model_cache = SentenceTransformer(model_name)

        embedding = _model_cache.encode(text).tolist()
        logging.info(f"Generated local embedding with {len(embedding)} dimensions for query: {text[:50]}...")
        return embedding

    except Exception as e:
        logging.error(f"Failed to build query embedding: {str(e)}")
        raise


def run_similarity_search(vector: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Run similarity search against Qdrant collection using provided vector.

    Args:
        vector: Embedding vector to search for similar items
        top_k: Number of top results to return (default 5)

    Returns:
        List of similar records with similarity scores and metadata
    """
    try:
        client = connect_qdrant()

        # Perform similarity search using query_points (modern API)
        search_response = client.query_points(
            collection_name=Config.QDRANT_COLLECTION_NAME,
            query=vector,
            limit=top_k,
            with_payload=True,
            score_threshold=Config.SIMILARITY_THRESHOLD  # Use configured threshold
        )

        results = []
        for hit in search_response.points:
            payload = hit.payload or {}
            results.append({
                'id': hit.id,
                'score': hit.score,
                'payload': payload,
                'url': payload.get('url', 'N/A'),
                'chunk_text': payload.get('content', payload.get('chunk_text', 'N/A')),
                'module': payload.get('module', 'N/A'),
                'chapter': payload.get('chapter', 'N/A'),
                'heading': payload.get('section_title', payload.get('heading', 'N/A')),
                'timestamp': payload.get('timestamp', 'N/A')
            })

        logging.info(f"Similarity search returned {len(results)} results with top score: {results[0]['score'] if results else 0}")
        return results

    except Exception as e:
        logging.error(f"Failed to run similarity search: {str(e)}")
        raise


def validate_results(records: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Validate retrieved records for completeness and quality.

    Args:
        records: List of records to validate

    Returns:
        Dictionary with validation results and statistics
    """
    validation_report = {
        'total_records': len(records),
        'valid_records': 0,
        'invalid_records': 0,
        'missing_fields': [],
        'field_completeness': {},
        'validation_passed': True
    }

    required_fields = ['url', 'chunk_text', 'module', 'chapter', 'id', 'timestamp']

    for record in records:
        record_valid = True
        missing_fields = []

        # Check for required fields
        for field in required_fields:
            if field not in record or record[field] is None or (isinstance(record[field], str) and record[field] == 'N/A'):
                missing_fields.append(field)
                record_valid = False

        # Update statistics
        if record_valid:
            validation_report['valid_records'] += 1
        else:
            validation_report['invalid_records'] += 1
            validation_report['missing_fields'].extend(missing_fields)

    # Calculate field completeness
    for field in required_fields:
        present_count = sum(1 for record in records if field in record and record[field] not in [None, 'N/A'])
        validation_report['field_completeness'][field] = {
            'present': present_count,
            'total': len(records),
            'percentage': (present_count / len(records)) * 100 if records else 0
        }

    # Overall validation status
    validation_report['validation_passed'] = validation_report['invalid_records'] == 0

    logging.info(f"Validation completed: {validation_report['valid_records']} valid records out of {validation_report['total_records']}")
    return validation_report


def inspect_collection() -> Dict[str, Any]:
    """
    Inspect Qdrant collection to get schema, stats, and configuration information.

    Returns:
        Dictionary with collection information including vector count, dimensions, etc.
    """
    try:
        client = connect_qdrant()

        # Get collection information
        collection_info = client.get_collection(collection_name=Config.QDRANT_COLLECTION_NAME)

        # Get record count
        count_info = client.count(
            collection_name=Config.QDRANT_COLLECTION_NAME,
            exact=True
        )

        inspection_report = {
            'collection_name': Config.QDRANT_COLLECTION_NAME,
            'vector_count': count_info.count,
            'vector_config': {
                'size': collection_info.config.params.vectors.size,
                'distance': collection_info.config.params.vectors.distance.value
            },
            'indexed_payload_fields': [],
            'shard_number': collection_info.config.params.shard_number,
            'status': collection_info.status.value
        }

        # Get payload schema
        payload_schema = collection_info.payload_schema
        inspection_report['indexed_payload_fields'] = list(payload_schema.keys()) if payload_schema else []

        logging.info(f"Collection inspection completed: {count_info.count} vectors in collection")
        return inspection_report

    except Exception as e:
        logging.error(f"Failed to inspect collection: {str(e)}")
        raise