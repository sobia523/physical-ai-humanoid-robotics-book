"""
Validation logic module for RAG Pipeline Retrieval Validation.

This module provides functions for validating retrieved records, embeddings,
and generating comprehensive validation reports.
"""

import logging
from typing import List, Dict, Any
from datetime import datetime

# Handle import based on execution context
try:
    from backend.config import Config
except ImportError:
    from config import Config
# Handle import based on execution context
try:
    from backend.utils.qdrant_helper import inspect_collection, fetch_sample_records, validate_results
except ImportError:
    from utils.qdrant_helper import inspect_collection, fetch_sample_records, validate_results

import json


def validate_embedding_dimensions(records: List[Dict[str, Any]], expected_dimension: int = 1024) -> Dict[str, Any]:
    """
    Validate that all embeddings have the expected dimension.

    Args:
        records: List of records with embedding vectors
        expected_dimension: Expected number of dimensions for embeddings

    Returns:
        Dictionary with validation results for embedding dimensions
    """
    validation_report = {
        'expected_dimension': expected_dimension,
        'valid_embeddings': 0,
        'invalid_embeddings': 0,
        'dimension_mismatches': [],
        'validation_passed': True
    }

    # For this validation, we would need to check the actual embedding vectors
    # Since we typically don't retrieve vectors in sample records for efficiency,
    # we'll validate based on collection inspection
    try:
        collection_info = inspect_collection()
        actual_dimension = collection_info['vector_config']['size']

        if actual_dimension == expected_dimension:
            validation_report['valid_embeddings'] = len(records)  # Assume all valid if dimension matches
            validation_report['validation_passed'] = True
        else:
            validation_report['invalid_embeddings'] = len(records)
            validation_report['dimension_mismatches'].append({
                'expected': expected_dimension,
                'actual': actual_dimension,
                'message': f'Embedding dimension mismatch: expected {expected_dimension}, got {actual_dimension}'
            })
            validation_report['validation_passed'] = False

        logging.info(f"Embedding dimension validation: {validation_report['valid_embeddings']} valid, {validation_report['invalid_embeddings']} invalid")
        return validation_report

    except Exception as e:
        logging.error(f"Failed to validate embedding dimensions: {str(e)}")
        validation_report['validation_passed'] = False
        return validation_report


def generate_validation_report() -> Dict[str, Any]:
    """
    Generate a comprehensive validation report for the Qdrant collection.

    Returns:
        Dictionary with complete validation report including all checks
    """
    start_time = datetime.now()
    logging.info("Starting comprehensive validation report generation")

    # Enhanced validation report structure
    report = {
        'report_id': f"validation_{start_time.strftime('%Y%m%d_%H%M%S')}",
        'timestamp': start_time.isoformat(),
        'version': '1.0',
        'configuration': {
            'qdrant_url': Config.QDRANT_URL,
            'collection_name': Config.QDRANT_COLLECTION_NAME,
            'sample_limit': Config.SAMPLE_RECORD_LIMIT,
            'similarity_top_k': Config.SIMILARITY_TOP_K,
            'similarity_threshold': Config.SIMILARITY_THRESHOLD,
            'cohere_model': Config.COHERE_MODEL
        },
        'collection_overview': {},
        'connection_validation': {},
        'sample_records_validation': {},
        'embedding_validation': {},
        'metadata_integrity': {},
        'retrieval_validation': {},
        'semantic_search_validation': {},
        'content_alignment_validation': {},
        'completeness_check': {},
        'diagnostics': {
            'errors': [],
            'warnings': [],
            'recommendations': []
        },
        'summary': {
            'total_records_inspected': 0,
            'validation_passed': False,
            'execution_time_seconds': 0,
            'overall_score': 0.0,
            'validation_status': 'failed'  # 'passed', 'partial', 'failed'
        }
    }

    try:
        # Connection validation
        logging.info("Performing connection validation...")
        report['connection_validation'] = run_connection_test()
        if not report['connection_validation']['connection_successful']:
            report['diagnostics']['errors'].append("Failed to connect to Qdrant")
            report['summary']['validation_status'] = 'failed'
            end_time = datetime.now()
            execution_time = (end_time - start_time).total_seconds()
            report['summary']['execution_time_seconds'] = execution_time
            return report

        # Collection inspection
        logging.info("Performing collection inspection...")
        report['collection_overview'] = inspect_collection()

        # Fetch and validate sample records
        logging.info("Fetching and validating sample records...")
        sample_records = fetch_sample_records(limit=Config.SAMPLE_RECORD_LIMIT)
        report['sample_records_validation'] = validate_results(sample_records)
        report['summary']['total_records_inspected'] = len(sample_records)

        # Validate embedding dimensions
        logging.info("Validating embedding dimensions...")
        report['embedding_validation'] = validate_embedding_dimensions(
            sample_records,
            expected_dimension=report['collection_overview']['vector_config']['size']
        )

        # Metadata integrity check
        logging.info("Checking metadata integrity...")
        report['metadata_integrity'] = check_metadata_integrity(sample_records)

        # Run retrieval test
        logging.info("Running retrieval test...")
        report['retrieval_validation'] = run_retrieval_test()

        # Run semantic search validation
        logging.info("Running semantic search validation...")
        report['semantic_search_validation'] = test_semantic_search_functionality()

        # Check content-context alignment
        logging.info("Checking content-context alignment...")
        if report['retrieval_validation']['results_count'] > 0:
            # Get the results from retrieval test for alignment check
            try:
                from backend.utils.qdrant_helper import build_query_embedding, run_similarity_search
            except ImportError:
                from utils.qdrant_helper import build_query_embedding, run_similarity_search
            query_embedding = build_query_embedding("Physical AI Humanoid Robotics")
            test_results = run_similarity_search(query_embedding, top_k=Config.SIMILARITY_TOP_K)
            report['content_alignment_validation'] = check_content_context_alignment(test_results)

        # Completeness check
        logging.info("Performing completeness check...")
        report['completeness_check'] = perform_completeness_check(report)

        # Generate summary and diagnostics
        validation_results = [
            report['connection_validation']['connection_successful'],
            report['sample_records_validation']['validation_passed'],
            report['embedding_validation']['validation_passed'],
            report['metadata_integrity']['validation_passed'],
            report['retrieval_validation']['results_valid'],
            report['semantic_search_validation']['semantic_search_functional']
        ]

        passed_checks = sum(validation_results)
        total_checks = len(validation_results)
        overall_score = (passed_checks / total_checks) * 100 if total_checks > 0 else 0

        report['summary']['overall_score'] = round(overall_score, 2)
        report['summary']['validation_passed'] = all(validation_results)
        report['summary']['validation_status'] = 'passed' if report['summary']['validation_passed'] else ('partial' if passed_checks > 0 else 'failed')

        # Generate diagnostics
        if not report['connection_validation']['connection_successful']:
            report['diagnostics']['errors'].append("Qdrant connection failed")
        if not report['sample_records_validation']['validation_passed']:
            report['diagnostics']['errors'].append("Sample records validation failed")
        if not report['embedding_validation']['validation_passed']:
            report['diagnostics']['errors'].append("Embedding validation failed")
        if not report['metadata_integrity']['validation_passed']:
            report['diagnostics']['errors'].append("Metadata integrity validation failed")
        if not report['retrieval_validation']['results_valid']:
            report['diagnostics']['errors'].append("Retrieval test failed")
        if not report['semantic_search_validation']['semantic_search_functional']:
            report['diagnostics']['errors'].append("Semantic search validation failed")

        # Add recommendations
        if report['metadata_integrity']['records_with_missing_metadata'] > 0:
            report['diagnostics']['recommendations'].append("Some records have missing metadata fields")
        if report['semantic_search_validation']['overall_success_rate'] < 80:
            report['diagnostics']['recommendations'].append("Semantic search success rate is below 80% - consider reviewing embeddings or query patterns")

        end_time = datetime.now()
        execution_time = (end_time - start_time).total_seconds()
        report['summary']['execution_time_seconds'] = execution_time

        logging.info(f"Validation report completed in {execution_time:.2f} seconds. Overall score: {overall_score:.2f}%. Validation status: {report['summary']['validation_status']}")
        return report

    except Exception as e:
        logging.error(f"Failed to generate validation report: {str(e)}")
        report['diagnostics']['errors'].append(f"Exception during validation: {str(e)}")
        report['summary']['validation_passed'] = False
        report['summary']['validation_status'] = 'failed'
        end_time = datetime.now()
        execution_time = (end_time - start_time).total_seconds()
        report['summary']['execution_time_seconds'] = execution_time
        return report


def perform_completeness_check(validation_report: Dict[str, Any]) -> Dict[str, Any]:
    """
    Perform a completeness check on the validation results.

    Args:
        validation_report: The full validation report to analyze

    Returns:
        Dictionary with completeness check results
    """
    completeness_report = {
        'total_expected_checks': 8,  # Based on the validation requirements
        'completed_checks': 0,
        'missing_checks': [],
        'completeness_percentage': 0.0,
        'completeness_status': 'incomplete'
    }

    # Check for required sections in the validation report
    required_sections = [
        'collection_overview',
        'connection_validation',
        'sample_records_validation',
        'embedding_validation',
        'metadata_integrity',
        'retrieval_validation',
        'semantic_search_validation',
        'summary'
    ]

    for section in required_sections:
        if section in validation_report and validation_report[section]:
            completeness_report['completed_checks'] += 1
        else:
            completeness_report['missing_checks'].append(section)

    completeness_report['completeness_percentage'] = (
        completeness_report['completed_checks'] / len(required_sections)
    ) * 100

    if completeness_report['completed_checks'] == len(required_sections):
        completeness_report['completeness_status'] = 'complete'
    elif completeness_report['completed_checks'] >= len(required_sections) * 0.8:
        completeness_report['completeness_status'] = 'mostly_complete'
    else:
        completeness_report['completeness_status'] = 'incomplete'

    return completeness_report


def run_similarity_queries(test_queries: List[str] = None, top_k: int = None) -> Dict[str, Any]:
    """
    Run similarity queries with test phrases against Qdrant collection.

    Args:
        test_queries: List of test query phrases to run (uses default if not provided)
        top_k: Number of top results to return (uses config default if not provided)

    Returns:
        Dictionary with similarity query results
    """
    if test_queries is None:
        from backend.test_queries import get_sample_test_queries
        test_queries = get_sample_test_queries()

    if top_k is None:
        top_k = Config.SIMILARITY_TOP_K

    query_report = {
        'total_queries': len(test_queries),
        'queries_executed': 0,
        'query_results': [],
        'queries_successful': 0,
        'queries_failed': 0,
        'average_response_time': 0.0,
        'error_details': []
    }

    try:
        from backend.utils.qdrant_helper import build_query_embedding, run_similarity_search
        import time

        total_response_time = 0

        for i, query in enumerate(test_queries):
            query_start_time = time.time()
            try:
                # Build query embedding
                query_embedding = build_query_embedding(query)

                # Run similarity search
                results = run_similarity_search(query_embedding, top_k=top_k)

                query_execution_time = time.time() - query_start_time
                total_response_time += query_execution_time

                query_result = {
                    'query_id': i,
                    'query_text': query,
                    'results_count': len(results),
                    'results': results,
                    'execution_time': query_execution_time,
                    'query_successful': True,
                    'error': None
                }

                query_report['queries_successful'] += 1

            except Exception as e:
                query_execution_time = time.time() - query_start_time
                total_response_time += query_execution_time

                query_result = {
                    'query_id': i,
                    'query_text': query,
                    'results_count': 0,
                    'results': [],
                    'execution_time': query_execution_time,
                    'query_successful': False,
                    'error': str(e)
                }

                query_report['queries_failed'] += 1
                query_report['error_details'].append({
                    'query_id': i,
                    'query_text': query,
                    'error': str(e)
                })

            query_report['query_results'].append(query_result)
            query_report['queries_executed'] += 1

        query_report['average_response_time'] = total_response_time / len(test_queries) if test_queries else 0

        logging.info(f"Similarity queries completed: {query_report['queries_successful']} successful, {query_report['queries_failed']} failed")
        return query_report

    except Exception as e:
        logging.error(f"Failed to run similarity queries: {str(e)}")
        query_report['error_details'].append({
            'query_id': 'all',
            'query_text': 'all queries',
            'error': str(e)
        })
        return query_report


def check_relevance_score(results: List[Dict[str, Any]], query_text: str = "") -> Dict[str, Any]:
    """
    Check relevance of search results to the original query.

    Args:
        results: List of search results with scores and content
        query_text: Original query text for context

    Returns:
        Dictionary with relevance checking results
    """
    relevance_report = {
        'total_results': len(results),
        'results_above_threshold': 0,
        'results_below_threshold': 0,
        'average_score': 0.0,
        'max_score': 0.0,
        'min_score': float('inf') if results else 0,
        'relevance_metrics': [],
        'semantic_relevance': True,  # Will be updated based on analysis
        'relevance_accuracy': 0.0
    }

    if not results:
        relevance_report['semantic_relevance'] = False
        logging.info("Relevance check: No results to evaluate")
        return relevance_report

    total_score = 0
    for result in results:
        score = result.get('score', 0)
        chunk_text = result.get('chunk_text', '').lower()
        query_terms = query_text.lower().split()

        # Calculate semantic relevance based on content overlap and score
        term_match_count = sum(1 for term in query_terms if term in chunk_text)
        term_relevance = term_match_count / len(query_terms) if query_terms else 0

        relevance_detail = {
            'record_id': result.get('id'),
            'score': score,
            'content_relevance': term_relevance,
            'semantic_relevance': score >= Config.SIMILARITY_THRESHOLD,
            'url': result.get('url', 'N/A')
        }

        relevance_report['relevance_metrics'].append(relevance_detail)

        if score >= Config.SIMILARITY_THRESHOLD:
            relevance_report['results_above_threshold'] += 1
        else:
            relevance_report['results_below_threshold'] += 1

        total_score += score
        relevance_report['max_score'] = max(relevance_report['max_score'], score)
        relevance_report['min_score'] = min(relevance_report['min_score'], score)

    relevance_report['average_score'] = total_score / len(results) if results else 0
    relevance_report['min_score'] = relevance_report['min_score'] if relevance_report['min_score'] != float('inf') else 0
    relevance_report['relevance_accuracy'] = (relevance_report['results_above_threshold'] / len(results)) * 100

    # Overall relevance assessment
    relevance_report['semantic_relevance'] = (
        relevance_report['results_above_threshold'] > 0 and
        relevance_report['average_score'] >= Config.SIMILARITY_THRESHOLD
    )

    logging.info(f"Relevance check: {relevance_report['results_above_threshold']} results above threshold out of {len(results)} total")
    return relevance_report


def retrieve_sample_records(limit: int = None) -> Dict[str, Any]:
    """
    Retrieve sample records from Qdrant for validation purposes.

    Args:
        limit: Number of sample records to retrieve (uses config default if not specified)

    Returns:
        Dictionary with retrieval results and sample records
    """
    if limit is None:
        limit = Config.SAMPLE_RECORD_LIMIT

    retrieval_report = {
        'limit_requested': limit,
        'records_retrieved': 0,
        'sample_records': [],
        'retrieval_successful': False,
        'error': None
    }

    try:
        from backend.utils.qdrant_helper import fetch_sample_records

        sample_records = fetch_sample_records(limit=limit)
        retrieval_report['records_retrieved'] = len(sample_records)
        retrieval_report['sample_records'] = sample_records
        retrieval_report['retrieval_successful'] = True

        logging.info(f"Sample record retrieval: {len(sample_records)} records retrieved successfully")
        return retrieval_report

    except Exception as e:
        logging.error(f"Sample record retrieval failed: {str(e)}")
        retrieval_report['error'] = str(e)
        return retrieval_report


def check_metadata_integrity(records: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Check integrity of metadata fields in retrieved records.

    Args:
        records: List of records to check for metadata integrity

    Returns:
        Dictionary with metadata integrity validation results
    """
    integrity_report = {
        'total_records': len(records),
        'records_with_complete_metadata': 0,
        'records_with_missing_metadata': 0,
        'missing_field_counts': {},
        'common_field_issues': [],
        'validation_passed': True
    }

    required_fields = ['url', 'chunk_text', 'module', 'chapter', 'id', 'timestamp']

    for record in records:
        missing_fields = []
        for field in required_fields:
            if field not in record or record[field] is None or (isinstance(record[field], str) and record[field] in ['', 'N/A', 'None']):
                missing_fields.append(field)

        if missing_fields:
            integrity_report['records_with_missing_metadata'] += 1
            integrity_report['validation_passed'] = False
            for field in missing_fields:
                if field not in integrity_report['missing_field_counts']:
                    integrity_report['missing_field_counts'][field] = 0
                integrity_report['missing_field_counts'][field] += 1
        else:
            integrity_report['records_with_complete_metadata'] += 1

    # Identify common issues
    for field, count in integrity_report['missing_field_counts'].items():
        if count > len(records) * 0.5:  # If more than 50% of records are missing this field
            integrity_report['common_field_issues'].append({
                'field': field,
                'missing_count': count,
                'percentage': (count / len(records)) * 100,
                'severity': 'high'
            })

    logging.info(f"Metadata integrity check: {integrity_report['records_with_complete_metadata']} records with complete metadata out of {len(records)} total")
    return integrity_report


def detect_missing_or_corrupted_records(records: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Detect missing or corrupted records in sampled results.

    Args:
        records: List of records to check for missing or corrupted data

    Returns:
        Dictionary with missing/corrupted record detection results
    """
    detection_report = {
        'total_records': len(records),
        'valid_records': 0,
        'missing_records': 0,
        'corrupted_records': 0,
        'missing_record_details': [],
        'corrupted_record_details': [],
        'integrity_score': 0.0,
        'integrity_passed': True
    }

    for record in records:
        record_id = record.get('id', 'unknown')
        is_missing = False
        is_corrupted = False
        corruption_details = []

        # Check for missing critical fields
        required_fields = ['url', 'chunk_text', 'id']
        missing_fields = []

        for field in required_fields:
            value = record.get(field)
            if value is None or (isinstance(value, str) and (value == '' or value == 'N/A' or value == 'None')):
                missing_fields.append(field)

        if missing_fields:
            is_missing = True
            detection_report['missing_record_details'].append({
                'record_id': record_id,
                'missing_fields': missing_fields
            })

        # Check for corrupted content
        chunk_text = record.get('chunk_text', '')
        url = record.get('url', '')

        # Check for content corruption indicators
        if chunk_text and len(chunk_text) < 10:  # Suspiciously short content
            corruption_details.append('suspiciously_short_content')
            is_corrupted = True

        if url and not url.startswith(('http://', 'https://')):
            corruption_details.append('invalid_url_format')
            is_corrupted = True

        # Check for common corruption patterns
        if chunk_text and all(c in [' ', '\n', '\t', '\r'] for c in chunk_text):
            corruption_details.append('only_whitespace_content')
            is_corrupted = True

        if chunk_text and len(set(chunk_text)) < 5:  # Very low character diversity
            corruption_details.append('low_character_diversity')
            is_corrupted = True

        if is_corrupted:
            detection_report['corrupted_record_details'].append({
                'record_id': record_id,
                'corruption_details': corruption_details
            })

        if not is_missing and not is_corrupted:
            detection_report['valid_records'] += 1
        elif is_missing:
            detection_report['missing_records'] += 1
        elif is_corrupted:
            detection_report['corrupted_records'] += 1

    if len(records) > 0:
        detection_report['integrity_score'] = (detection_report['valid_records'] / len(records)) * 100
        detection_report['integrity_passed'] = detection_report['missing_records'] == 0 and detection_report['corrupted_records'] == 0

    logging.info(f"Record integrity check: {detection_report['valid_records']} valid, {detection_report['missing_records']} missing, {detection_report['corrupted_records']} corrupted out of {len(records)} total")
    return detection_report


def check_content_context_alignment(results: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Check that source URLs and content match semantic context.

    Args:
        results: List of search results with URLs and content

    Returns:
        Dictionary with content-context alignment verification results
    """
    alignment_report = {
        'total_results': len(results),
        'results_with_aligned_content': 0,
        'results_with_misaligned_content': 0,
        'alignment_details': [],
        'overall_alignment_score': 0.0,
        'context_alignment_passed': True
    }

    for result in results:
        url = result.get('url', 'N/A')
        chunk_text = result.get('chunk_text', '')
        content_module = result.get('module', 'unknown')

        # Analyze if the content matches the expected context based on URL
        url_lower = url.lower()
        chunk_lower = chunk_text.lower()

        # Determine expected content based on URL path
        expected_content_indicators = []
        if 'ros2-nervous-system' in url_lower:
            expected_content_indicators = ['ros2', 'nervous', 'system', 'robot', 'operating', 'node', 'topic', 'service']
        elif 'digital-twin-sim' in url_lower:
            expected_content_indicators = ['digital', 'twin', 'simulation', 'model', '3d', 'physics', 'environment']
        elif 'humanoid' in url_lower or 'robotics' in url_lower:
            expected_content_indicators = ['robot', 'humanoid', 'ai', 'motion', 'control', 'actuator']

        # Count how many expected indicators are in the content
        indicator_matches = sum(1 for indicator in expected_content_indicators if indicator in chunk_lower)

        # Calculate alignment score
        alignment_score = indicator_matches / len(expected_content_indicators) if expected_content_indicators else 1.0
        is_aligned = alignment_score > 0.3  # At least 30% of expected indicators should be present

        alignment_detail = {
            'record_id': result.get('id'),
            'url': url,
            'content_module': content_module,
            'expected_indicators': expected_content_indicators,
            'indicator_matches': indicator_matches,
            'alignment_score': alignment_score,
            'is_aligned': is_aligned
        }

        if is_aligned:
            alignment_report['results_with_aligned_content'] += 1
        else:
            alignment_report['results_with_misaligned_content'] += 1
            alignment_report['context_alignment_passed'] = False

        alignment_report['alignment_details'].append(alignment_detail)

    if len(results) > 0:
        alignment_report['overall_alignment_score'] = (alignment_report['results_with_aligned_content'] / len(results)) * 100

    logging.info(f"Content-context alignment: {alignment_report['results_with_aligned_content']} aligned out of {len(results)} total")
    return alignment_report


def verify_url_mapping(records: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Verify mapping between retrieved records and original URLs.

    Args:
        records: List of records to verify URL mapping for

    Returns:
        Dictionary with URL mapping verification results
    """
    verification_report = {
        'total_records': len(records),
        'records_with_valid_urls': 0,
        'records_with_invalid_urls': 0,
        'url_validation_details': [],
        'mapping_accuracy': 0.0,
        'validation_passed': True
    }

    import re
    from urllib.parse import urlparse

    for record in records:
        url = record.get('url', record.get('payload', {}).get('url', 'N/A'))

        # Check if URL is valid
        try:
            result = urlparse(url)
            is_valid_url = all([result.scheme, result.netloc]) and url != 'N/A'
        except Exception:
            is_valid_url = False

        # Check if URL follows expected pattern for the documentation
        expected_patterns = [
            r'https?://.*physical-ai-humanoid-robotics-book.*',
            r'.*/docs/modules/.*',
            r'.*/docs/modules/ros2-nervous-system/.*',
            r'.*/docs/modules/digital-twin-sim/.*'
        ]

        url_matches_pattern = any(re.search(pattern, url) for pattern in expected_patterns) if is_valid_url else False

        validation_detail = {
            'record_id': record.get('id', 'unknown'),
            'url': url,
            'is_valid_url': is_valid_url,
            'matches_expected_pattern': url_matches_pattern
        }

        if is_valid_url and url_matches_pattern:
            verification_report['records_with_valid_urls'] += 1
        else:
            verification_report['records_with_invalid_urls'] += 1
            verification_report['validation_passed'] = False

        verification_report['url_validation_details'].append(validation_detail)

    if len(records) > 0:
        verification_report['mapping_accuracy'] = (verification_report['records_with_valid_urls'] / len(records)) * 100

    logging.info(f"URL mapping verification: {verification_report['records_with_valid_urls']} valid URLs out of {len(records)} total")
    return verification_report


def run_connection_test() -> Dict[str, Any]:
    """
    Test connection to Qdrant and validate basic connectivity.

    Returns:
        Dictionary with connection test results
    """
    test_report = {
        'connection_attempted': True,
        'connection_successful': False,
        'collection_info': None,
        'error': None
    }

    try:
        # Handle import based on execution context
        try:
            from backend.utils.qdrant_helper import connect_qdrant, inspect_collection
        except ImportError:
            from utils.qdrant_helper import connect_qdrant, inspect_collection

        # Test basic connection
        client = connect_qdrant()
        test_report['connection_successful'] = True

        # Get collection info
        collection_info = inspect_collection()
        test_report['collection_info'] = collection_info

        logging.info(f"Connection test successful: Connected to {collection_info['collection_name']} with {collection_info['vector_count']} vectors")
        return test_report

    except Exception as e:
        logging.error(f"Connection test failed: {str(e)}")
        test_report['error'] = str(e)
        return test_report


def test_semantic_search_functionality() -> Dict[str, Any]:
    """
    Test semantic search functionality with various query types.

    Returns:
        Dictionary with comprehensive semantic search test results
    """
    test_report = {
        'tests_performed': 0,
        'tests_passed': 0,
        'tests_failed': 0,
        'test_results': [],
        'overall_success_rate': 0.0,
        'semantic_search_functional': True,
        'error': None
    }

    try:
        # Handle import based on execution context
        try:
            from backend.test_queries import get_sample_test_queries
        except ImportError:
            from test_queries import get_sample_test_queries

        try:
            from backend.utils.qdrant_helper import build_query_embedding, run_similarity_search, validate_results
        except ImportError:
            from utils.qdrant_helper import build_query_embedding, run_similarity_search, validate_results

        # Get various types of test queries
        test_queries = get_sample_test_queries(limit=10)

        for i, query in enumerate(test_queries):
            single_test_result = {
                'test_id': i,
                'query_text': query,
                'query_type': 'general',
                'results_count': 0,
                'top_score': 0,
                'results_above_threshold': 0,
                'semantic_relevance': False,
                'test_passed': False,
                'error': None
            }

            # Determine query type based on content
            query_lower = query.lower()
            if 'ros2' in query_lower:
                single_test_result['query_type'] = 'ros2'
            elif 'digital' in query_lower or 'twin' in query_lower or 'simulation' in query_lower:
                single_test_result['query_type'] = 'digital_twin'
            elif 'humanoid' in query_lower or 'robot' in query_lower:
                single_test_result['query_type'] = 'robotics'

            try:
                # Build query embedding
                query_embedding = build_query_embedding(query)

                # Run similarity search
                results = run_similarity_search(query_embedding, top_k=Config.SIMILARITY_TOP_K)

                # Check relevance
                relevance_check = check_relevance_score(results, query)

                # Check content-context alignment
                alignment_check = check_content_context_alignment(results)

                single_test_result['results_count'] = len(results)
                single_test_result['top_score'] = results[0]['score'] if results else 0
                single_test_result['results_above_threshold'] = relevance_check['results_above_threshold']
                single_test_result['semantic_relevance'] = relevance_check['semantic_relevance']
                single_test_result['content_alignment'] = alignment_check['context_alignment_passed']

                # Test passes if we get results and they're relevant
                single_test_result['test_passed'] = (
                    len(results) > 0 and
                    relevance_check['results_above_threshold'] > 0 and
                    relevance_check['semantic_relevance']
                )

                if single_test_result['test_passed']:
                    test_report['tests_passed'] += 1
                else:
                    test_report['tests_failed'] += 1

            except Exception as e:
                single_test_result['error'] = str(e)
                test_report['tests_failed'] += 1

            test_report['test_results'].append(single_test_result)
            test_report['tests_performed'] += 1

        test_report['overall_success_rate'] = (test_report['tests_passed'] / test_report['tests_performed']) * 100 if test_report['tests_performed'] > 0 else 0
        test_report['semantic_search_functional'] = test_report['tests_passed'] > 0

        logging.info(f"Semantic search tests: {test_report['tests_passed']} passed out of {test_report['tests_performed']} total")
        return test_report

    except Exception as e:
        logging.error(f"Semantic search functionality test failed: {str(e)}")
        test_report['error'] = str(e)
        return test_report


def run_retrieval_test() -> Dict[str, Any]:
    """
    Run a simple retrieval test to verify similarity search functionality.

    Returns:
        Dictionary with retrieval test results
    """
    test_report = {
        'test_query': 'Physical AI Humanoid Robotics',
        'test_performed': False,
        'results_count': 0,
        'top_score': 0,
        'results_valid': False,
        'error': None
    }

    try:
        # Handle import based on execution context
        try:
            from backend.utils.qdrant_helper import build_query_embedding, run_similarity_search
        except ImportError:
            from utils.qdrant_helper import build_query_embedding, run_similarity_search

        # Build query embedding
        query_embedding = build_query_embedding(test_report['test_query'])

        # Run similarity search
        results = run_similarity_search(query_embedding, top_k=Config.SIMILARITY_TOP_K)

        test_report['test_performed'] = True
        test_report['results_count'] = len(results)
        test_report['top_score'] = results[0]['score'] if results else 0
        test_report['results_valid'] = len(results) > 0 and results[0]['score'] >= Config.SIMILARITY_THRESHOLD

        logging.info(f"Retrieval test completed: {len(results)} results with top score {results[0]['score'] if results else 0}")
        return test_report

    except Exception as e:
        logging.error(f"Retrieval test failed: {str(e)}")
        test_report['error'] = str(e)
        return test_report


def save_validation_report(report: Dict[str, Any], filename: str = None) -> str:
    """
    Save validation report to a JSON file.

    Args:
        report: Validation report dictionary to save
        filename: Optional filename to save report (default: validation_report_YYYYMMDD_HHMMSS.json)

    Returns:
        Path to saved file
    """
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"validation_report_{timestamp}.json"

    filepath = f"backend/{filename}"

    try:
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)

        logging.info(f"Validation report saved to {filepath}")
        return filepath
    except Exception as e:
        logging.error(f"Failed to save validation report: {str(e)}")
        raise