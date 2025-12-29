import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import and run the RAG pipeline with force_rerun
from main import run_rag_pipeline

print("Running RAG pipeline with force_rerun=True to ensure all URLs are processed...")
result = run_rag_pipeline(force_rerun=True)

print(f"\nPipeline completed with results:")
print(f"- Processed URLs: {result.get('processed_urls', 0)}")
print(f"- Failed URLs: {result.get('failed_urls', 0)}")
print(f"- Total chunks: {result.get('total_chunks', 0)}")
print(f"- Duration: {result.get('duration_seconds', 0)} seconds")

if result.get('errors'):
    print(f"\nErrors encountered:")
    for error in result.get('errors', []):
        print(f"  - {error}")
else:
    print("\nNo errors encountered during pipeline execution.")