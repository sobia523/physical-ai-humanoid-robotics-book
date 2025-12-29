# Data Model: RAG Pipeline Retrieval Validation

## Entities

### Qdrant Collection
- **Name**: String identifier for the collection
- **Description**: Container for vector embeddings and metadata
- **Schema**: Predefined structure with vector dimensions and payload fields
- **Vector Dimensions**: 1024 (for Cohere embed-english-v3.0 model)
- **Distance Metric**: Cosine distance
- **Record Count**: Number of stored embeddings

### Sample Record
- **ID**: Unique identifier for the record in Qdrant
- **Vector**: 1024-dimensional embedding vector
- **Payload**: Dictionary containing metadata fields
  - **url**: Source URL of the original content
  - **module**: Module identifier from the documentation structure
  - **chapter**: Chapter identifier from the documentation structure
  - **section_title**: Title of the section from which content was extracted
  - **chunk_text**: The actual text content of the chunk
  - **token_length**: Number of tokens in the chunk text
  - **timestamp**: When the record was stored
  - **source**: Identifier of the source pipeline that created the record

### Similarity Query
- **Query Text**: Original text used for similarity search
- **Query Vector**: 1024-dimensional embedding of the query text
- **Top-K**: Number of results to retrieve
- **Threshold**: Minimum similarity score for inclusion

### Search Result
- **Record**: Reference to a Sample Record
- **Score**: Similarity score between query and result
- **Rank**: Position in the ranked results list
- **Metadata**: Copy of the payload from the Sample Record

### Validation Report
- **Timestamp**: When the validation was performed
- **Connection Status**: Success/failure of Qdrant connection
- **Collection Stats**: Statistics about the collection
- **Sample Records Retrieved**: Count and details of sample records
- **Search Results**: Results from similarity queries
- **Data Integrity Checks**: Results of validation checks
- **Diagnostic Messages**: Any error or warning messages
- **Overall Status**: Pass/fail status of the validation

## Relationships

- Qdrant Collection **contains** multiple Sample Records
- Similarity Query **generates** multiple Search Results
- Search Results **reference** Sample Records
- Validation Report **includes** Sample Records, Search Results, and diagnostic information

## Validation Rules

1. **Sample Record Completeness**: Each record must have all required metadata fields (url, chunk_text, timestamp)
2. **Vector Dimensionality**: All vectors must have 1024 dimensions
3. **URL Format**: URLs must be properly formatted and valid
4. **Token Length**: Token length must be consistent with the chunk text
5. **Similarity Score Range**: Scores must be between 0 and 1 for cosine similarity
6. **Record Count**: Collection must have at least 200 records as per success criteria