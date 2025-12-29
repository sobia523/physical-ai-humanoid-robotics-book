import os
import requests
import xml.etree.ElementTree as ET
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

def main():
    sitemap_url = "https://physical-ai-humanoid-robotics-book-rust.vercel.app/sitemap.xml"
    res = requests.get(sitemap_url)
    root = ET.fromstring(res.content)
    namespaces = {'ns': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
    all_urls = [loc.text for loc in root.findall('.//ns:loc', namespaces)]
    docs_urls = [u for u in all_urls if "/docs/" in u]
    
    client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"), https=True)
    collection_name = "book_embeddings"
    
    points_count = client.count(collection_name).count
    
    present_urls = set()
    next_offset = None
    while True:
        points, next_offset = client.scroll(collection_name=collection_name, limit=100, offset=next_offset)
        for point in points:
            u = point.payload.get('url')
            if u: present_urls.add(u)
        if next_offset is None: break
        
    ingested_docs = [u for u in docs_urls if u in present_urls]
    
    print(f"DEBUG_START")
    print(f"TOTAL_DOCS_IN_SITEMAP: {len(docs_urls)}")
    print(f"TOTAL_POINTS_IN_QDRANT: {points_count}")
    print(f"UNIQUE_URLS_IN_QDRANT: {len(present_urls)}")
    print(f"DOCS_INGESTED_SUCCESSFULLY: {len(ingested_docs)}")
    print(f"DEBUG_END")

if __name__ == "__main__":
    main()
