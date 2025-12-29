import os
import requests
import xml.etree.ElementTree as ET
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

def fetch_sitemap_urls(url):
    try:
        response = requests.get(url)
        response.raise_for_status()
        root = ET.fromstring(response.content)
        namespaces = {'ns': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
        urls = [loc.text for loc in root.findall('.//ns:loc', namespaces)]
        return [u for u in urls if "/docs/" in u]
    except Exception as e:
        print(f"Error fetching sitemap: {e}")
        return []

def main():
    target_sitemap = "https://physical-ai-humanoid-robotics-book-rust.vercel.app/sitemap.xml"
    sitemap_urls = fetch_sitemap_urls(target_sitemap)
    
    client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"), https=True)
    collection_name = "book_embeddings"
    
    # 1. Verification of Coverage
    present_urls = set()
    next_offset = None
    sample_content = []
    
    print(f"Auditing Qdrant collection: {collection_name}...")
    while True:
        points, next_offset = client.scroll(collection_name=collection_name, limit=100, offset=next_offset, with_payload=True)
        for point in points:
            url = point.payload.get('url')
            if url:
                present_urls.add(url)
                if len(sample_content) < 3 and 'content' in point.payload:
                    sample_content.append({
                        'title': point.payload.get('title'),
                        'section': point.payload.get('section_title'),
                        'text': point.payload.get('content')[:200] + "..."
                    })
        if next_offset is None:
            break
            
    ingested_count = sum(1 for u in sitemap_urls if u in present_urls)
    missing_urls = [u for u in sitemap_urls if u not in present_urls]
    
    print("\n" + "="*50)
    print("FINAL INGESTION CONFIRMATION")
    print("="*50)
    print(f"Target URLs in Sitemap:   {len(sitemap_urls)}")
    print(f"Actually Ingested URLs:   {ingested_count}")
    print(f"Coverage Percentage:      {(ingested_count/len(sitemap_urls)*100):.2f}%")
    print(f"Total Vectors in Qdrant:  {client.count(collection_name).count}")
    print("="*50)
    
    print("\nCONTENT VALIDATION (SAMPLE):")
    for i, sample in enumerate(sample_content):
        print(f"\nSample {i+1}:")
        print(f"Book/Module Title: {sample['title']}")
        print(f"Section:           {sample['section']}")
        print(f"Content Preview:   {sample['text']}")
        
    if missing_urls:
        print(f"\nMISSING FROM SITEMAP ({len(missing_urls)}):")
        for m in missing_urls[:5]:
            print(f" - {m}")
        if len(missing_urls) > 5: print("   ... and more.")

if __name__ == "__main__":
    main()