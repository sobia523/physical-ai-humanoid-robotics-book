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

def map_path_to_url(local_path):
    path_str = str(local_path).replace('\\', '/')
    if 'docs/' in path_str:
        rel_path = path_str.split('docs/', 1)[1]
    else:
        rel_path = path_str
    rel_path = os.path.splitext(rel_path)[0]
    parts = rel_path.split('/')
    if len(parts) > 1 and parts[0] == "modules" and len(parts) > 2:
        parts[1] = re.sub(r'^\d+-', '', parts[1])
    rel_path = "/".join(parts)
    return f"{BASE_URL}/docs/{rel_path}"

def main():
    qdrant_client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    
    test_file = Path("docs/modules/001-ros2-nervous-system/chapters/01/introduction.md")
    print(f"Testing with: {test_file}")
    
    title, sections = parse_markdown(test_file)
    url = map_path_to_url(test_file)
    print(f"Mapped URL: {url}")
    print(f"Title: {title}")
    
    all_chunks = []
    parts = test_file.parts
    module = re.sub(r'^\d+-', '', parts[2])
    chapter = parts[4] + "/" + parts[5].replace('.md', '')
    
    for section in sections:
        text_chunks = chunk_text(section["content"])
        for chunk in text_chunks:
            all_chunks.append({
                "chunk_text": chunk,
                "metadata": {
                    "url": url,
                    "title": title,
                    "section_title": section["title"],
                    "module": module,
                    "chapter": chapter,
                    "token_length": count_tokens(chunk),
                    "source": "local_markdown_test"
                }
            })
            
    print(f"Total chunks: {len(all_chunks)}")
    
    # Just embed and upload the first batch
    batch = all_chunks[:BATCH_SIZE]
    texts = [c["chunk_text"] for c in batch]
    response = cohere_client.embed(texts=texts, model=COHERE_MODEL, input_type="search_document")
    embeddings = response.embeddings
    
    points = [models.PointStruct(id=str(uuid.uuid4()), vector=emb, payload=c["metadata"]) 
              for c, emb in zip(batch, embeddings)]
              
    qdrant_client.upload_points(collection_name=QDRANT_COLLECTION_NAME, points=points, wait=True)
    print(f"Successfully uploaded {len(points)} points.")

if __name__ == "__main__":
    main()
