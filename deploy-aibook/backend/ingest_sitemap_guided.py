import os
import re
import uuid
import time
import logging
import requests
import xml.etree.ElementTree as ET
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
SITEMAP_URL = "https://physical-ai-humanoid-robotics-book-rust.vercel.app/sitemap.xml"
BASE_URL = "https://physical-ai-humanoid-robotics-book-rust.vercel.app"
QDRANT_COLLECTION_NAME = "book_embeddings"
EMBEDDING_MODEL_NAME = "all-MiniLM-L6-v2"
VECTOR_SIZE = 384 

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

    title = ""
    title_match = re.search(r'^# (.*)$', content, re.MULTILINE)
    if title_match:
        title = title_match.group(1).strip()

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
            if not line.startswith('# '):
                current_section["content"] += line + "\n"
    if current_section["content"].strip():
        sections.append(current_section)
    return title, sections

def fetch_sitemap_urls(url):
    try:
        response = requests.get(url)
        response.raise_for_status()
        root = ET.fromstring(response.content)
        namespaces = {'ns': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
        urls = [loc.text for loc in root.findall('.//ns:loc', namespaces)]
        return [u for u in urls if "/docs/" in u]
    except Exception as e:
        logger.error(f"Error fetching sitemap: {e}")
        return []

def get_local_files_map(docs_dir):
    """Creates a map of sanitized paths to local file paths."""
    file_map = {}
    docs_path = Path(docs_dir)
    for md_file in docs_path.rglob("*.md"):
        # Create a key that looks like a URL part
        rel_path = md_file.relative_to(docs_path)
        parts = list(rel_path.parts)
        
        # Sanitize parts: remove numeric prefixes (001-vla -> vla)
        sanitized_parts = [re.sub(r'^\d+-', '', p) for p in parts]
        # Remove extension from last part
        sanitized_parts[-1] = sanitized_parts[-1].replace('.md', '')
        
        key = "/".join(sanitized_parts).lower()
        file_map[key] = md_file
    return file_map

def main():
    # 1. Initialize
    logger.info("Initializing Professional Sitemap-Guided Ingestion...")
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        https=True,
        timeout=60
    )
    model = SentenceTransformer(EMBEDDING_MODEL_NAME)

    # 2. Fetch Sitemap
    logger.info(f"Fetching URLs from {SITEMAP_URL}...")
    sitemap_urls = fetch_sitemap_urls(SITEMAP_URL)
    logger.info(f"Found {len(sitemap_urls)} documentation URLs in sitemap.")

    # 3. Create Local File Map
    local_map = get_local_files_map("docs/modules")
    logger.info(f"Scanned {len(local_map)} local Markdown files.")

    # 4. Fresh Collection
    logger.info(f"Wiping and recreating collection {QDRANT_COLLECTION_NAME}...")
    qdrant_client.recreate_collection(
        collection_name=QDRANT_COLLECTION_NAME,
        vectors_config=models.VectorParams(size=VECTOR_SIZE, distance=models.Distance.COSINE),
    )
    qdrant_client.create_payload_index(QDRANT_COLLECTION_NAME, "url", models.PayloadSchemaType.KEYWORD)

    # 5. Map & Chunk
    all_chunks = []
    mapped_count = 0
    
    for url in sitemap_urls:
        # PROFESSIONAL FILTER: Exclude system/config pages that look irrelevant to book content
        lower_url = url.lower()
        if any(keyword in lower_url for keyword in ["/configs/", "/summary", "/templates", "/architecture-diagram", "/chapter-template"]):
            logger.info(f"Filtering out non-book URL: {url}")
            continue

        # Extract the relevant part for mapping
        match = re.search(r'/docs/modules/(.*)$', url)
        if not match:
            logger.warning(f"Skipping non-module URL: {url}")
            continue
            
        url_path = match.group(1).lower().rstrip('/')
        
        # Try direct match
        md_file = local_map.get(url_path)
        
        # Try fuzzy match (sometimes /docs/modules/vla-systems maps to /docs/modules/vla-systems/index or intro)
        if not md_file:
            for k in local_map:
                if k.startswith(url_path) and (k.endswith('/introduction') or k.endswith('/index')):
                    md_file = local_map[k]
                    break
        
        if md_file:
            mapped_count += 1
            logger.info(f"MATCH FOUND: URL [{url}] -> FILE [{md_file}]")
            title, sections = parse_markdown(md_file)
            
            # Metadata determines module/chapter from local path
            parts = list(md_file.parts)
            module = re.sub(r'^\d+-', '', parts[2]) if len(parts) > 2 else "unknown"
            
            for section in sections:
                text_chunks = chunk_text(section["content"])
                for text in text_chunks:
                    all_chunks.append({
                        "text": text,
                        "metadata": {
                            "url": url, # Canonical URL from Sitemap
                            "title": title,
                            "section_title": section["title"],
                            "module": module,
                            "content": text, # Added actual text content for search results
                            "source": "sitemap_local_sync"
                        }
                    })
        else:
            logger.warning(f"No local file for sitemap URL: {url}")

    logger.info(f"Successfully mapped {mapped_count}/{len(sitemap_urls)} URLs.")
    logger.info(f"Total chunks to process: {len(all_chunks)}")

    # 6. Upload
    BATCH_SIZE = 128
    for i in range(0, len(all_chunks), BATCH_SIZE):
        batch = all_chunks[i:i + BATCH_SIZE]
        texts = [c["text"] for c in batch]
        
        embeddings = model.encode(texts)
        points = []
        for j, (chunk, embedding) in enumerate(zip(batch, embeddings)):
            points.append(models.PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding.tolist(),
                payload=chunk["metadata"]
            ))
            
        # Retry logic
        for attempt in range(3):
            try:
                qdrant_client.upsert(collection_name=QDRANT_COLLECTION_NAME, points=points)
                break
            except Exception as e:
                if attempt == 2: raise
                time.sleep(5)
        
        logger.info(f"Uploaded batch {i//BATCH_SIZE + 1}/{(len(all_chunks)-1)//BATCH_SIZE + 1}")

    logger.info("PROFESSIONAL SYNC COMPLETE!")

if __name__ == "__main__":
    main()
