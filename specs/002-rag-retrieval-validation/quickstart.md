# Quickstart: RAG Pipeline Retrieval Validation

## Overview
This guide provides instructions for setting up and running the RAG Pipeline Retrieval Validation system. This system validates that stored embeddings and metadata in Qdrant can be successfully retrieved, reconstructed, and inspected to confirm ingestion quality and pipeline correctness.

## Prerequisites
- Python 3.8 or higher
- Access to Qdrant Cloud collection created in Spec-1
- Valid Qdrant API credentials
- Valid Cohere API key (same as used in Spec-1)
- Git

## Setup

1. **Clone the repository** (if not already done):
   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. **Create a virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

   If requirements.txt doesn't exist, install the required packages:
   ```bash
   pip install qdrant-client cohere python-dotenv tiktoken requests
   ```

4. **Configure environment variables**:
   Create a `.env` file in the backend directory with the following:
   ```
   QDRANT_URL=<your-qdrant-url>
   QDRANT_API_KEY=<your-qdrant-api-key>
   QDRANT_COLLECTION_NAME=book_embeddings
   COHERE_API_KEY=<your-cohere-api-key>
   COHERE_MODEL=embed-english-v3.0
   SAMPLE_RECORD_LIMIT=10
   SIMILARITY_TOP_K=5
   SIMILARITY_THRESHOLD=0.5
   VALIDATION_TIMEOUT=300
   ```

## Running the Validation

1. **Navigate to the backend directory**:
   ```bash
   cd backend
   ```

2. **Run the validation script**:
   ```bash
   python validation_runner.py
   ```

3. **Run with additional options**:
   ```bash
   python validation_runner.py --verbose --output validation_report.json --test-queries 10 --validate-chunk-count 200
   ```

4. **For a quick connection test only**:
   ```bash
   python validate_connection.py
   ```

## Validation Steps Performed

The validation system performs the following comprehensive checks:

1. **Connection Validation**: Tests connectivity to Qdrant Cloud
2. **Collection Inspection**: Verifies collection schema and statistics
3. **Sample Retrieval**: Fetches sample records with metadata
4. **Embedding Validation**: Confirms embedding dimensions match expected model
5. **Metadata Integrity**: Validates completeness of metadata fields
6. **Similarity Search**: Tests retrieval functionality with test queries
7. **Content Alignment**: Verifies URLs and content match semantic context
8. **Comprehensive Reporting**: Generates detailed validation report

## Expected Output

After running the validation, you should see:

- Connection status to Qdrant
- Collection statistics (number of vectors, dimensions, etc.)
- Sample records retrieved with metadata
- Similarity search results for test queries
- Content-context alignment verification
- Validation report with pass/fail status and overall score
- Diagnostic information and recommendations

## Command Line Options

- `--verbose` or `-v`: Enable verbose logging
- `--output` or `-o`: Output file for validation report (JSON format)
- `--test-queries`: Number of test queries to run (default: 5)
- `--validate-chunk-count`: Minimum expected chunk count (default: 200)

## Validation Report Structure

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

## Success Criteria

The validation is considered successful when:

- Connection to Qdrant is established
- Collection contains ≥200 stored chunks
- Embedding dimensions match 1024-dim Cohere model
- All required metadata fields are present and valid
- Similarity search returns relevant results
- Content and URLs match expected semantic context
- Overall validation score is ≥80%

## Troubleshooting

- **Connection errors**: Verify QDRANT_URL and QDRANT_API_KEY in your .env file
- **Authentication errors**: Ensure your Qdrant API key has read permissions
- **No records found**: Verify that the collection name matches the one from Spec-1
- **Embedding dimension mismatch**: Ensure you're using the same Cohere model as Spec-1
- **Rate limiting**: If using a Cohere trial key, consider reducing the number of test queries
- **Missing metadata**: Some fields may be missing if the original documents had incomplete information

## Next Steps

After successful validation:
1. Review the validation report for any issues
2. Address any failed validation checks
3. Run the validation again to confirm fixes
4. Proceed to the next phase of your RAG pipeline development