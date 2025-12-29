import os
import sys
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
import requests
from bs4 import BeautifulSoup
import tiktoken
import cohere
import time
import logging

# Load environment variables
load_dotenv()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    https=True
)

# Initialize Cohere client
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))

collection_name = "book_embeddings"

# Specific ROS2 URLs to ensure ingestion
target_urls = [
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/01/introduction",
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/02/ros2-fundamentals",
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/03/python-agents-bridging",
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/04/urdf-humanoids",
    "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/05/practical-integration"
]

def crawl_url(url):
    """Crawl a URL and return HTML content"""
    try:
        response = requests.get(url, timeout=30)
        response.raise_for_status()
        return response.text
    except Exception as e:
        print(f"Error crawling {url}: {str(e)}")
        return None

def extract_content_from_html(html, url):
    """Extract content from HTML using BeautifulSoup"""
    if not html:
        return ""

    soup = BeautifulSoup(html, 'html.parser')

    # Remove script and style elements
    for script in soup(["script", "style"]):
        script.decompose()

    # Extract main content - look for main content containers
    main_content = soup.find('main') or soup.find('article') or soup.find('div', class_='main') or soup.find('div', class_='content') or soup.body

    if main_content:
        # Get text content
        text = main_content.get_text()

        # Clean up text
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)

        return text
    else:
        return soup.get_text()

def count_tokens(text):
    """Count tokens in text using tiktoken"""
    encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
    return len(encoding.encode(text))

def chunk_text(text, max_tokens=800, overlap_tokens=200):
    """Chunk text into pieces with specified token limits"""
    encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")

    # Encode the text
    tokens = encoding.encode(text)

    chunks = []
    start_idx = 0
    token_count = len(tokens)

    while start_idx < token_count:
        # Calculate end index
        end_idx = start_idx + max_tokens

        # If we're near the end, include the rest
        if end_idx >= token_count:
            end_idx = token_count
        else:
            # Try to break at sentence boundary
            chunk_tokens = tokens[start_idx:end_idx]
            chunk_text = encoding.decode(chunk_tokens)

            # Find a good breaking point (try to break at sentence or paragraph)
            last_period = chunk_text.rfind('.', 0, max_tokens//2)
            if last_period != -1:
                end_idx = start_idx + len(encoding.encode(chunk_text[:last_period+1]))

        # Extract the chunk
        chunk_tokens = tokens[start_idx:end_idx]
        chunk_text = encoding.decode(chunk_tokens)

        if chunk_text.strip():
            chunks.append({
                'text': chunk_text,
                'token_count': len(chunk_tokens)
            })

        # Move start index forward, accounting for overlap
        start_idx = end_idx - overlap_tokens if end_idx < token_count else end_idx

        # Ensure we don't get stuck in an infinite loop
        if start_idx <= start_idx:  # If we didn't advance
            start_idx += max_tokens

    return chunks

def generate_embeddings(texts):
    """Generate embeddings using Cohere API"""
    try:
        response = cohere_client.embed(
            texts=texts,
            model="embed-english-v3.0",
            input_type="search_document"
        )
        return response.embeddings
    except Exception as e:
        print(f"Error generating embeddings: {str(e)}")
        return None

import uuid

def store_in_qdrant(url, module, chapter, section_title, chunk_text, embedding):
    """Store a single chunk in Qdrant"""
    try:
        # Create payload with metadata
        payload = {
            'url': url,
            'module': module,
            'chapter': chapter,
            'section_title': section_title,
            'chunk_text': chunk_text,
            'source': 'manual_ingestion',
            'timestamp': time.time(),
            'token_length': count_tokens(chunk_text)
        }

        # Upload to Qdrant with UUID
        point_id = str(uuid.uuid4())
        qdrant_client.upsert(
            collection_name=collection_name,
            points=[
                models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=payload
                )
            ]
        )

        return True
    except Exception as e:
        print(f"Error storing in Qdrant: {str(e)}")
        return False

print("MANUAL INGESTION: Processing specific ROS2 chapter URLs...")
print("=" * 70)

for url in target_urls:
    print(f"\nProcessing: {url}")

    # 1. Crawl the URL
    html_content = crawl_url(url)
    if not html_content:
        print(f"  - Failed to crawl URL")
        continue

    # 2. Extract content
    content = extract_content_from_html(html_content, url)
    if not content.strip():
        print(f"  - No content extracted")
        continue

    print(f"  - Extracted content ({len(content)} chars)")

    # 3. Chunk the content
    chunks = chunk_text(content)
    print(f"  - Created {len(chunks)} chunks")

    # 4. Generate embeddings for all chunks
    chunk_texts = [chunk['text'] for chunk in chunks]
    embeddings = generate_embeddings(chunk_texts)

    if not embeddings:
        print(f"  - Failed to generate embeddings")
        continue

    print(f"  - Generated {len(embeddings)} embeddings")

    # 5. Extract module and chapter from URL
    module = "ros2-nervous-system"
    chapter = url.split('/')[-1]  # Get the last part of the URL

    # 6. Store each chunk with its embedding
    successful_stores = 0
    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        success = store_in_qdrant(
            url=url,
            module=module,
            chapter=chapter,
            section_title=f"Section {i+1}",
            chunk_text=chunk['text'],
            embedding=embedding
        )

        if success:
            successful_stores += 1

    print(f"  - Successfully stored {successful_stores} out of {len(chunks)} chunks")

print("\n" + "=" * 70)
print("Manual ingestion of specific ROS2 URLs completed!")

# Verify that the URLs are now in Qdrant
print("\nVERIFICATION: Checking if URLs are now in Qdrant...")

found_count = 0
for url in target_urls:
    try:
        # Try a different approach to search - use scroll with filter
        points, next_page = qdrant_client.scroll(
            collection_name=collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="url",
                        match=models.MatchValue(value=url)
                    )
                ]
            ),
            limit=1
        )

        if len(points) > 0:
            print(f"[CONFIRMED] {url} - Found {len(points)} chunks in Qdrant")
            found_count += 1
        else:
            print(f"[MISSING] {url} - Not found in Qdrant")
    except Exception as e:
        print(f"[ERROR] Could not verify {url}: {str(e)}")

print(f"\nSUMMARY: {found_count} out of {len(target_urls)} URLs confirmed in Qdrant")