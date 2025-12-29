import os
import re
import uuid
import time
import logging
from pathlib import Path
from dotenv import load_dotenv
import tiktoken
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Load environment variables
load_dotenv()

# Configuration
CHUNK_SIZE = 800
OVERLAP_SIZE = 200
BASE_URL = "https://physical-ai-humanoid-robotics-book-rust.vercel.app"
QDRANT_COLLECTION_NAME = "book_embeddings"
COHERE_MODEL = "embed-english-v3.0"
BATCH_SIZE = 96

# Logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def count_tokens(text):
    encoding = tiktoken.get_encoding("cl100k_base")
    return len(encoding.encode(text))

def chunk_text(text, chunk_size=CHUNK_SIZE, overlap_size=OVERLAP_SIZE):
    sentences = re.split(r'[.!?]+\s+', text)
    chunks = []
    current_chunk = ""
    current_token_count = 0

    for sentence in sentences:
        sentence_token_count = count_tokens(sentence)
        if current_token_count + sentence_token_count > chunk_size:
            if current_chunk.strip():
                chunks.append(current_chunk.strip())
            current_chunk = sentence
            current_token_count = sentence_token_count
        else:
            current_chunk = (current_chunk + " " + sentence).strip()
            current_token_count += sentence_token_count

    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    if overlap_size > 0 and len(chunks) > 1:
        overlapped_chunks = []
        for i, chunk in enumerate(chunks):
            if i == 0:
                overlapped_chunks.append(chunk)
            else:
                prev_chunk_tokens = chunks[i-1].split()
                overlap_text = " ".join(prev_chunk_tokens[-overlap_size:])
                overlapped_chunks.append((overlap_text + " " + chunk).strip())
        return overlapped_chunks
    return chunks

def parse_markdown(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Simple MD parser
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
    parts = rel_path.split('/')
    if len(parts) > 1 and parts[0] == "modules" and len(parts) > 2:
        parts[1] = re.sub(r'^\d+-', '', parts[1])
    
    rel_path = "/".join(parts)
    return f"{BASE_URL}/docs/{rel_path}"

def main():
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        prefer_grpc=True
    )
    
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    
    docs_dir = Path("docs/modules")
    all_chunks = []
    
    logger.info(f"Scanning {docs_dir} for Markdown files...")
    
    for md_file in docs_dir.rglob("*.md"):
        logger.info(f"Parsing {md_file}")
        title, sections = parse_markdown(md_file)
        url = map_path_to_url(md_file)
        
        # Determine module and chapter from path
        parts = md_file.parts
        module = "unknown"
        chapter = "unknown"
        
        # index 0 is docs, index 1 is modules, index 2 is module-name
        if len(parts) > 2:
            module = re.sub(r'^\d+-', '', parts[2])
        if len(parts) > 4: # docs/modules/module/chapters/XX/name.md
            chapter = parts[4] + "/" + parts[5].replace('.md', '')
        elif len(parts) > 3:
             chapter = parts[3].replace('.md', '')
        
        for section in sections:
            text_chunks = chunk_text(section["content"])
            for i, chunk in enumerate(text_chunks):
                all_chunks.append({
                    "chunk_text": chunk,
                    "metadata": {
                        "url": url,
                        "title": title,
                        "section_title": section["title"],
                        "module": module,
                        "chapter": chapter,
                        "token_length": count_tokens(chunk),
                        "timestamp": time.time(),
                        "source": "local_markdown"
                    }
                })

    logger.info(f"Collected total {len(all_chunks)} chunks. Starting embedding and upload in batches of {BATCH_SIZE}...")

    success_count = 0
    for i in range(0, len(all_chunks), BATCH_SIZE):
        batch = all_chunks[i:i + BATCH_SIZE]
        texts = [c["chunk_text"] for c in batch]
        
        try:
            logger.info(f"Processing batch {i//BATCH_SIZE + 1}/{(len(all_chunks)-1)//BATCH_SIZE + 1}...")
            response = cohere_client.embed(
                texts=texts,
                model=COHERE_MODEL,
                input_type="search_document"
            )
            embeddings = response.embeddings
            
            points = []
            for j, (chunk, embedding) in enumerate(zip(batch, embeddings)):
                points.append(models.PointStruct(
                    id=str(uuid.uuid4()),
                    vector=embedding,
                    payload=chunk["metadata"]
                ))
                
            qdrant_client.upload_points(
                collection_name=QDRANT_COLLECTION_NAME,
                points=points,
                wait=True
            )
            success_count += len(batch)
            logger.info(f"Successfully uploaded {len(batch)} points.")
            
        except Exception as e:
            logger.error(f"Error processing batch starting at {i}: {e}")
            # Try a small sleep to avoid hitting rate limits too hard if we're close
            time.sleep(2)
            continue

    logger.info(f"Ingestion complete. Total points uploaded: {success_count}/{len(all_chunks)}")

if __name__ == "__main__":
    main()
