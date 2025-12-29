"""
Standalone validation runner for RAG Pipeline Retrieval Validation.

This script provides a complete end-to-end validation of the Qdrant collection,
including connection testing, sample retrieval, similarity search, and comprehensive reporting.
"""

import argparse
import sys
import logging
from datetime import datetime
import json
from pathlib import Path

# Add backend to path to import modules
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Handle imports based on execution context
try:
    from backend.utils.qdrant_helper import (
        connect_qdrant,
        inspect_collection,
        fetch_sample_records,
        build_query_embedding,
        run_similarity_search
    )
    from backend.utils.validation import (
        generate_validation_report,
        save_validation_report,
        run_connection_test,
        run_similarity_queries,
        test_semantic_search_functionality
    )
    from backend.test_queries import get_sample_test_queries
    from backend.config import Config
except ImportError:
    from utils.qdrant_helper import (
        connect_qdrant,
        inspect_collection,
        fetch_sample_records,
        build_query_embedding,
        run_similarity_search
    )
    from utils.validation import (
        generate_validation_report,
        save_validation_report,
        run_connection_test,
        run_similarity_queries,
        test_semantic_search_functionality
    )
    from test_queries import get_sample_test_queries
    from config import Config


def setup_logging(verbose: bool = False):
    """Set up logging for the validation runner."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('validation_runner.log'),
            logging.StreamHandler(sys.stdout)
        ]
    )


def print_validation_results(report: dict):
    """Print validation results in a user-friendly format."""
    print("\n" + "="*80)
    print("RAG PIPELINE RETRIEVAL VALIDATION REPORT")
    print("="*80)

    print(f"Report ID: {report.get('report_id', 'N/A')}")
    print(f"Timestamp: {report.get('timestamp', 'N/A')}")
    print(f"Status: {report.get('summary', {}).get('validation_status', 'N/A').upper()}")
    print(f"Overall Score: {report.get('summary', {}).get('overall_score', 0.0):.2f}%")
    print(f"Execution Time: {report.get('summary', {}).get('execution_time_seconds', 0):.2f}s")

    print("\nCONFIGURATION:")
    config = report.get('configuration', {})
    print(f"  Qdrant Collection: {config.get('collection_name', 'N/A')}")
    print(f"  Sample Limit: {config.get('sample_limit', 'N/A')}")
    print(f"  Top-K: {config.get('similarity_top_k', 'N/A')}")
    print(f"  Threshold: {config.get('similarity_threshold', 'N/A')}")

    print("\nCOLLECTION OVERVIEW:")
    collection = report.get('collection_overview', {})
    print(f"  Vector Count: {collection.get('vector_count', 'N/A')}")
    print(f"  Vector Size: {collection.get('vector_config', {}).get('size', 'N/A')}")
    print(f"  Distance: {collection.get('vector_config', {}).get('distance', 'N/A')}")

    print("\nVALIDATION RESULTS:")
    summary = report.get('summary', {})
    print(f"  Total Records Inspected: {summary.get('total_records_inspected', 0)}")
    print(f"  Validation Passed: {'PASS' if summary.get('validation_passed', False) else 'FAIL'}")

    print("\nDETAILED RESULTS:")
    connection_valid = report.get('connection_validation', {}).get('connection_successful', False)
    print(f"  Connection Test: {'PASS' if connection_valid else 'FAIL'}")

    sample_valid = report.get('sample_records_validation', {}).get('validation_passed', False)
    print(f"  Sample Records Validation: {'PASS' if sample_valid else 'FAIL'}")

    embedding_valid = report.get('embedding_validation', {}).get('validation_passed', False)
    print(f"  Embedding Validation: {'PASS' if embedding_valid else 'FAIL'}")

    metadata_valid = report.get('metadata_integrity', {}).get('validation_passed', False)
    print(f"  Metadata Integrity: {'PASS' if metadata_valid else 'FAIL'}")

    retrieval_valid = report.get('retrieval_validation', {}).get('results_valid', False)
    print(f"  Retrieval Test: {'PASS' if retrieval_valid else 'FAIL'}")

    semantic_valid = report.get('semantic_search_validation', {}).get('semantic_search_functional', False)
    print(f"  Semantic Search: {'PASS' if semantic_valid else 'FAIL'}")

    # Print diagnostics
    diagnostics = report.get('diagnostics', {})
    errors = diagnostics.get('errors', [])
    warnings = diagnostics.get('warnings', [])
    recommendations = diagnostics.get('recommendations', [])

    if errors:
        print(f"\nERRORS ({len(errors)}):")
        for error in errors:
            print(f"  - {error}")

    if warnings:
        print(f"\nWARNINGS ({len(warnings)}):")
        for warning in warnings:
            print(f"  - {warning}")

    if recommendations:
        print(f"\nRECOMMENDATIONS ({len(recommendations)}):")
        for rec in recommendations:
            print(f"  - {rec}")

    print("\n" + "="*80)


def run_end_to_end_validation() -> dict:
    """Run the complete end-to-end validation workflow."""
    print("Starting end-to-end validation...")

    # Generate comprehensive validation report
    report = generate_validation_report()

    return report


def run_comprehensive_tests():
    """Run comprehensive tests including all validation aspects."""
    print("Running comprehensive validation tests...")

    results = {
        'connection_test': run_connection_test(),
        'sample_retrieval': True,  # This is part of the main validation
        'similarity_queries': run_similarity_queries(get_sample_test_queries()),
        'semantic_search': test_semantic_search_functionality()
    }

    return results


def main():
    parser = argparse.ArgumentParser(description='RAG Pipeline Retrieval Validation Runner')
    parser.add_argument('--verbose', '-v', action='store_true', help='Enable verbose logging')
    parser.add_argument('--output', '-o', type=str, help='Output file for validation report (JSON format)')
    parser.add_argument('--test-queries', type=int, default=5, help='Number of test queries to run (default: 5)')
    parser.add_argument('--validate-chunk-count', type=int, default=200, help='Minimum expected chunk count (default: 200)')

    args = parser.parse_args()

    # Set up logging
    setup_logging(args.verbose)

    print("RAG Pipeline Retrieval Validation Runner")
    print("========================================")

    try:
        # Validate configuration first
        print("Validating configuration...")
        Config.validate_config()
        print("SUCCESS: Configuration validation passed")

        # Run end-to-end validation
        validation_report = run_end_to_end_validation()

        # Print results
        print_validation_results(validation_report)

        # Check chunk count requirement
        expected_chunks = args.validate_chunk_count
        actual_chunks = validation_report.get('collection_overview', {}).get('vector_count', 0)
        chunk_count_ok = actual_chunks >= expected_chunks

        print(f"\nChunk Count Validation:")
        print(f"  Expected: >={expected_chunks}")
        print(f"  Actual: {actual_chunks}")
        print(f"  Status: {'SUCCESS: PASS' if chunk_count_ok else 'FAIL'}")

        # Save report if requested
        if args.output:
            output_path = save_validation_report(validation_report, args.output)
            print(f"\nValidation report saved to: {output_path}")

        # Determine overall success
        validation_passed = validation_report.get('summary', {}).get('validation_passed', False)
        overall_success = validation_passed and chunk_count_ok

        print(f"\nOVERALL VALIDATION STATUS: {'PASS' if overall_success else 'FAIL'}")

        return 0 if overall_success else 1

    except Exception as e:
        logging.error(f"Validation failed with error: {str(e)}")
        print(f"\nVALIDATION FAILED: {str(e)}")
        return 1


if __name__ == "__main__":
    sys.exit(main())