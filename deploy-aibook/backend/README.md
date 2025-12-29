# RAG Pipeline for Physical AI Humanoid Robotics Textbook

This repository contains the RAG (Retrieval-Augmented Generation) pipeline that processes content from the Physical AI Humanoid Robotics Textbook for use in a chatbot knowledge base.

## Overview

The RAG pipeline:
1. Crawls the deployed book URLs from the sitemap
2. Extracts structured content (titles, sections, paragraphs)
3. Chunks content into 500-1200 token segments with overlap
4. Generates embeddings using Cohere's embedding models
5. Stores embeddings with metadata in a Qdrant vector database

## Prerequisites

- Node.js 18+
- Python 3.8+
- npm or yarn package manager

## Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Navigate to the backend directory:
   ```bash
   cd backend
   ```

3. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Create a `.env` file in the backend directory with the following environment variables:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   ```

## Environment Variables

- `COHERE_API_KEY`: Your Cohere API key for generating text embeddings
- `QDRANT_API_KEY`: Your Qdrant Cloud API key
- `QDRANT_URL`: Your Qdrant Cloud cluster URL

## Running the Pipeline

### Development Mode
To run the pipeline in development mode:

```bash
python main.py
```

### Production Mode
To run the pipeline with specific parameters:

```bash
# Run with default settings
python main.py

# Run with specific sitemap URL and maximum pages
python main.py --sitemap-url https://example.com/sitemap.xml --max-pages 10
```

## Configuration

The pipeline can be configured through the following settings in `main.py`:

- `CHUNK_SIZE`: Maximum tokens per chunk (default: 800)
- `OVERLAP_SIZE`: Overlap tokens between chunks (default: 200)
- `SITEMAP_URL`: URL of the sitemap to crawl (default: from config)
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: "book_embeddings")
- `COHERE_MODEL`: Cohere model to use for embeddings (default: "embed-english-v3.0")

## Features

- **Idempotent Execution**: The pipeline can be safely re-run without duplicating content
- **Resume Capability**: Can resume from a specific URL if interrupted
- **Caching**: Embeddings are cached to avoid duplicate API calls
- **Error Handling**: Graceful degradation when individual documents fail
- **Retry Logic**: Automatic retries with exponential backoff for API calls
- **Comprehensive Logging**: Detailed logs for monitoring and debugging
- **Verification Reports**: Automated reports confirming successful processing

## Verification

After running the pipeline, a verification report is automatically generated that includes:
- Total URLs processed
- Total content chunks created
- Qdrant collection statistics
- Sample records from the vector database
- Error summary

## Troubleshooting

### Common Issues

1. **API Rate Limits**: If you encounter rate limit errors, the pipeline has built-in retry logic with exponential backoff.

2. **Missing Environment Variables**: Ensure all required environment variables are set before running the pipeline.

3. **Network Issues**: The pipeline handles network failures gracefully and continues processing other URLs.

### Checking Results

To verify that content was successfully stored in Qdrant:
1. Check the verification report generated after each run
2. Use Qdrant's web UI to inspect the collection
3. Run a sample similarity search to confirm embeddings are retrievable

## Architecture

The pipeline follows a modular architecture:
- **Crawling Layer**: Handles URL fetching and HTML content extraction
- **Processing Layer**: Content cleaning, chunking, and text preparation
- **Embedding Layer**: Cohere API integration for vector generation
- **Storage Layer**: Qdrant integration for vector storage with metadata
- **Orchestration Layer**: Main pipeline function connecting all components
- **Verification Layer**: Result validation and reporting

## Validation System

The validation system provides comprehensive testing of the RAG pipeline's retrieval capabilities, including:
- Connection testing to Qdrant Cloud
- Sample record retrieval and validation
- Similarity search functionality testing
- Metadata integrity verification
- Comprehensive validation reporting

### Components

- `qdrant_helper.py` - Qdrant connection and operations
- `validation.py` - Validation logic and reporting
- `config.py` - Configuration management
- `validation_runner.py` - Standalone validation runner
- `validate_connection.py` - Connection validation script
- `test_queries.py` - Test query phrases for validation

### Key Functions

- `connect_qdrant()` - Establish connection to Qdrant Cloud
- `fetch_sample_records(limit)` - Retrieve sample records with metadata
- `build_query_embedding(text)` - Create embedding vector for query text
- `run_similarity_search(vector, top_k)` - Perform similarity search in Qdrant
- `generate_validation_report()` - Create comprehensive validation report
- `validate_results(records)` - Validate retrieved records for completeness

### Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Ensure your `.env` file includes the following environment variables:
   ```env
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=book_embeddings
   COHERE_API_KEY=your_cohere_api_key
   COHERE_MODEL=embed-english-v3.0
   SAMPLE_RECORD_LIMIT=10
   SIMILARITY_TOP_K=5
   SIMILARITY_THRESHOLD=0.5
   VALIDATION_TIMEOUT=300
   ```

### Usage

#### Quick Validation Test

Run a basic connection test:
```bash
python validate_connection.py
```

#### Full Validation

Run the complete validation workflow:
```bash
python validation_runner.py
```

With additional options:
```bash
python validation_runner.py --verbose --output validation_report.json --test-queries 10 --validate-chunk-count 200
```

#### Command Line Options

- `--verbose` or `-v`: Enable verbose logging
- `--output` or `-o`: Output file for validation report (JSON format)
- `--test-queries`: Number of test queries to run (default: 5)
- `--validate-chunk-count`: Minimum expected chunk count (default: 200)

### Validation Process

The validation system performs the following checks:

1. **Connection Validation**: Tests connectivity to Qdrant Cloud
2. **Collection Inspection**: Verifies collection schema and statistics
3. **Sample Retrieval**: Fetches sample records with metadata
4. **Embedding Validation**: Confirms embedding dimensions match expected model
5. **Metadata Integrity**: Validates completeness of metadata fields
6. **Similarity Search**: Tests retrieval functionality with test queries
7. **Content Alignment**: Verifies URLs and content match semantic context
8. **Comprehensive Reporting**: Generates detailed validation report

### Validation Report Structure

The validation report includes:

- Configuration settings used
- Collection overview information
- Connection validation results
- Sample records validation
- Embedding validation results
- Metadata integrity checks
- Retrieval validation
- Semantic search validation
- Content alignment validation
- Completeness checks
- Diagnostic information (errors, warnings, recommendations)
- Summary with pass/fail status and execution metrics

### Success Criteria

The validation is considered successful when:

- Connection to Qdrant is established
- Collection contains ≥200 stored chunks
- Embedding dimensions match 1024-dim Cohere model
- All required metadata fields are present and valid
- Similarity search returns relevant results
- Content and URLs match expected semantic context
- Overall validation score is ≥80%

### Read-Only Operations

All validation operations are read-only and do not modify the Qdrant collection in any way.

## Success Criteria

The pipeline meets these success criteria:
- 100% of selected book URLs are processed without data loss
- Each document entry contains proper metadata (URL, section title, chunk text, token length, timestamp)
- Minimum 200+ content chunks are successfully embedded and indexed
- Embedding + storage pipeline is repeatable and executable via a single script
- Verification confirms sample records can be queried from Qdrant