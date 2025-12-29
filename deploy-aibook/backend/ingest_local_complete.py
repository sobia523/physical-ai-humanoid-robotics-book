import os
import re
import uuid
import time
import logging
from pathlib import Path
from dotenv import load_dotenv
import tiktoken
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Load environment variables
load_dotenv()

# Configuration
CHUNK_SIZE = 800
BASE_URL = "https://physical-ai-humanoid-robotics-book-rust.vercel.app"
QDRANT_COLLECTION_NAME = "book_embeddings"
EMBEDDING_MODEL_NAME = "all-MiniLM-L6-v2"
VECTOR_SIZE = 384  # for all-MiniLM-L6-v2

# Logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def count_tokens(text):
    encoding = tiktoken.get_encoding("cl100k_base")
    return len(encoding.encode(text))

def chunk_text(text, chunk_size=CHUNK_SIZE):
    # Split by double newlines or single newlines followed by a header
    # But for a local model, we can be simpler and just split by paragraphs/sections
    paragraphs = re.split(r'\n\s*\n', text)
    chunks = []
    current_chunk = ""
    current_token_count = 0

    for para in paragraphs:
        para_token_count = count_tokens(para)
        if current_token_count + para_token_count > chunk_size:
            if current_chunk.strip():
                chunks.append(current_chunk.strip())
            current_chunk = para
            current_token_count = para_token_count
        else:
            current_chunk = (current_chunk + "\n\n" + para).strip()
            current_token_count += para_token_count

    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks

def parse_markdown(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract title
    title = ""
    title_match = re.search(r'^# (.*)$', content, re.MULTILINE)
    if title_match:
        title = title_match.group(1).strip()

    # Extract sections
    sections = []
    current_section = {"title": "Introduction", "content": ""}
    
    lines = content.split('\n')
    for line in lines:
        header_match = re.search(r'^(#{2,6}) (.*)$', line)
        if header_match:
            if current_section["content"].strip():
                sections.append(current_section)
            current_section = {"title": header_match.group(2).strip(), "content": ""}
        else:
            if not line.startswith('# '): # Skip top-level title
                current_section["content"] += line + "\n"
    
    if current_section["content"].strip():
        sections.append(current_section)
        
    return title, sections

def map_path_to_url(local_path):
    # Example: docs/modules/001-ros2-nervous-system/chapters/01/introduction.md
    # -> /docs/modules/ros2-nervous-system/chapters/01/introduction
    path_str = str(local_path).replace('\\', '/')
    
    # Extract relative part from 'docs'
    if 'docs/' in path_str:
        rel_path = path_str.split('docs/', 1)[1]
    else:
        rel_path = path_str
        
    # Remove extension
    rel_path = os.path.splitext(rel_path)[0]
    
    # Strip numeric prefixes from module folders (e.g., 001-ros2 -> ros2)
    # Also handle the nested chapter folders
    parts = rel_path.split('/')
    sanitized_parts = []
    for part in parts:
        # Remove prefixes like 001-, 01-, name-etc becomes etc if etc is after digits
        # Actually, looking at the user request: /docs/modules/ros2-nervous-system/chapters/01/introduction
        # The chapter number '01' is preserved, but the module '001-ros2-nervous-system' becomes 'ros2-nervous-system'
        if part.startswith('0') and '-' in part:
             sanitized_parts.append(re.sub(r'^\d+-', '', part))
        else:
             sanitized_parts.append(part)
             
    rel_path = "/".join(sanitized_parts)
    return f"{BASE_URL}/docs/{rel_path}"

def main():
    # 1. Initialize Clients
    logger.info("Initializing Qdrant and Local Embedding Model...")
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        https=True,
        timeout=60 # Increase timeout to 60s
    )
    
    model = SentenceTransformer(EMBEDDING_MODEL_NAME)
    
    # 2. Recreate Collection - ONLY if explicitly requested or if it doesn't exist
    force_fresh = True # Set to True to wipe and start over
    
    exists = False
    try:
        qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
        exists = True
        logger.info(f"Collection {QDRANT_COLLECTION_NAME} already exists.")
    except:
        logger.info(f"Collection {QDRANT_COLLECTION_NAME} does not exist.")

    if force_fresh or not exists:
        logger.info(f"Recreating collection {QDRANT_COLLECTION_NAME} for {VECTOR_SIZE} dimensions...")
        qdrant_client.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=models.VectorParams(size=VECTOR_SIZE, distance=models.Distance.COSINE),
        )
        # Add payload indexes
        qdrant_client.create_payload_index(QDRANT_COLLECTION_NAME, "url", models.PayloadSchemaType.KEYWORD)
        qdrant_client.create_payload_index(QDRANT_COLLECTION_NAME, "module", models.PayloadSchemaType.KEYWORD)
    
    # Get existing URLs for idempotency
    existing_urls = set()
    if exists and not force_fresh:
        logger.info("Fetching existing URLs in Qdrant for skipped processing...")
        next_offset = None
        while True:
            points, next_offset = qdrant_client.scroll(
                collection_name=QDRANT_COLLECTION_NAME,
                with_payload=True,
                limit=100,
                offset=next_offset
            )
            for point in points:
                url = point.payload.get('url')
                if url:
                    existing_urls.add(url)
            if next_offset is None:
                break
        logger.info(f"Found {len(existing_urls)} existing URLs in Qdrant.")
    
    # 3. Scan Files
    docs_dir = Path("docs/modules")
    all_chunks = []
    
    logger.info(f"Scanning {docs_dir} for Markdown files...")
    md_files = list(docs_dir.rglob("*.md"))
    logger.info(f"Found {len(md_files)} Markdown files.")
    
    for md_file in md_files:
        url = map_path_to_url(md_file)
        if url in existing_urls:
            logger.info(f"Skipping {url} (already ingested)")
            continue
            
        title, sections = parse_markdown(md_file)
        
        # Determine module/chapter for metadata
        parts = list(md_file.parts)
        module = "unknown"
        chapter = "unknown"
        
        # docs is parts[0], modules is parts[1], module_name is parts[2]
        if len(parts) > 2:
            module = re.sub(r'^\d+-', '', parts[2])
            
        # Everything after module name is the path/chapter
        if len(parts) > 3:
            chapter = "/".join(parts[3:]).replace('.md', '')
        else:
            chapter = "index"
        
        for section in sections:
            text_chunks = chunk_text(section["content"])
            for chunk_text_content in text_chunks:
                all_chunks.append({
                    "text": chunk_text_content,
                    "metadata": {
                        "url": url,
                        "title": title,
                        "section_title": section["title"],
                        "module": module,
                        "chapter": chapter,
                        "token_length": count_tokens(chunk_text_content),
                        "source": "local_ingestion_v2"
                    }
                })

    logger.info(f"Total chunks to process: {len(all_chunks)}")

    # 4. Generate Embeddings & Upload
    # Local model is fast, we can process in larger batches
    BATCH_SIZE = 128
    total_uploaded = 0
    
    for i in range(0, len(all_chunks), BATCH_SIZE):
        batch = all_chunks[i:i + BATCH_SIZE]
        texts = [c["text"] for c in batch]
        
        logger.info(f"Embedding and uploading batch {i//BATCH_SIZE + 1}/{(len(all_chunks)-1)//BATCH_SIZE + 1}...")
        embeddings = model.encode(texts)
        
        points = []
        for j, (chunk, embedding) in enumerate(zip(batch, embeddings)):
            points.append(models.PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding.tolist(),
                payload=chunk["metadata"]
            ))
            
        # Retry logic for upsert
        max_retries = 3
        for attempt in range(max_retries):
            try:
                qdrant_client.upsert(
                    collection_name=QDRANT_COLLECTION_NAME,
                    points=points
                )
                break
            except Exception as e:
                if attempt == max_retries - 1:
                    logger.error(f"Failed to upload batch {i//BATCH_SIZE + 1} after {max_retries} attempts: {e}")
                    raise
                logger.warning(f"Upload attempt {attempt + 1} failed, retrying: {e}")
                time.sleep(5)
                
        total_uploaded += len(batch)

    logger.info(f"INGESTION COMPLETE!")
    logger.info(f"Total points stored: {total_uploaded}")
    logger.info(f"Total URLs processed: {len(md_files)}")
    
    # Simple verification scroll
    res = qdrant_client.scroll(collection_name=QDRANT_COLLECTION_NAME, limit=5, with_payload=True)
    logger.info(f"Sample stored URLs: {[p.payload.get('url') for p in res[0]]}")

if __name__ == "__main__":
    main()
