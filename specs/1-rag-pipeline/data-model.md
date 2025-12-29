# Data Model: RAG Pipeline

## Document Chunk
**Description**: Represents a text segment extracted from book content
**Fields**:
- url (string): Source page URL
- section_title (string): Title of the section/chapter
- chunk_text (string): The actual text content of the chunk
- token_length (integer): Number of tokens in the chunk
- embedding_vector (array[float]): The embedding vector representation
- timestamp (datetime): When the chunk was processed

## Embedding Vector
**Description**: Vector representation of text content for semantic search
**Fields**:
- vector (array[float]): Float array representing the embedding (dimension: 1024)
- model (string): The model used to generate the embedding
- text_hash (string): Hash of the original text for verification

## Qdrant Record
**Description**: Stored entry in the vector database with embedding and metadata
**Fields**:
- id (string): Unique identifier for the record
- vector (array[float]): The embedding vector
- payload (object): Contains metadata including url, section_title, chunk_text, token_length, timestamp
- score (float): Similarity score (for search results)

## Processing Log
**Description**: Record of pipeline execution including status and metrics
**Fields**:
- timestamp (datetime): When the processing started
- status (string): Current status (processing, completed, failed)
- processed_urls (integer): Count of URLs successfully processed
- total_chunks (integer): Total number of chunks created
- errors (array[object]): List of errors encountered during processing
- duration (float): Total processing time in seconds