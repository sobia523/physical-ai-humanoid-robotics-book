# Quickstart: RAG Pipeline

## Prerequisites

- Python 3.11+
- pip package manager
- UV package manager (or pip)

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Navigate to backend directory**
   ```bash
   mkdir -p backend
   cd backend
   ```

3. **Install dependencies**
   Using UV (recommended):
   ```bash
   uv init
   uv pip install requests beautifulsoup4 cohere qdrant-client tiktoken lxml python-dotenv
   ```

   Or using pip:
   ```bash
   pip install requests beautifulsoup4 cohere qdrant-client tiktoken lxml python-dotenv
   ```

4. **Set up environment variables**
   Create a `.env` file in the backend directory with:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_URL=your_qdrant_cluster_url
   ```

## Usage

1. **Run the pipeline**
   ```bash
   python main.py
   ```

2. **The pipeline will execute the following steps:**
   - Fetch book URLs from sitemap
   - Crawl and extract content from each URL
   - Chunk content into 500-1200 token segments
   - Generate embeddings using Cohere
   - Store embeddings in Qdrant Cloud collection
   - Generate verification report

## Configuration

The pipeline can be configured by modifying these parameters in main.py:
- `CHUNK_SIZE`: Maximum tokens per chunk (default: 800)
- `OVERLAP_SIZE`: Overlap tokens between chunks (default: 200)
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: "book_embeddings")
- `SITEMAP_URL`: URL of the book sitemap (default: "https://physical-ai-humanoid-robotics-book-rust.vercel.app/sitemap.xml")

## Verification

After running the pipeline:
1. Check the console output for processing statistics
2. Verify that 200+ chunks were processed successfully
3. Query the Qdrant collection to confirm embeddings were stored
4. Review the generated log file for any errors