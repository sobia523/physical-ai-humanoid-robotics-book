import os
import requests
import xml.etree.ElementTree as ET
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

SITEMAP_URL = "https://physical-ai-humanoid-robotics-book-rust.vercel.app/sitemap.xml"
COLLECTION_NAME = "book_embeddings"

def fetch_sitemap_urls(url):
    try:
        response = requests.get(url)
        response.raise_for_status()
        root = ET.fromstring(response.content)
        # Handle namespaces
        namespaces = {'ns': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
        urls = [loc.text for loc in root.findall('.//ns:loc', namespaces)]
        return urls
    except Exception as e:
        print(f"Error fetching sitemap: {e}")
        return []

def main():
    # 1. Fetch Sitemap URLs
    all_urls = fetch_sitemap_urls(SITEMAP_URL)
    docs_urls = [url for url in all_urls if "/docs/" in url]
    
    # 2. Add manual fallback URLs
    manual_urls = [
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/01/introduction",
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/02/ros2-fundamentals",
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/03/python-agents-bridging",
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/04/urdf-humanoids",
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/05/practical-integration"
    ]
    for url in manual_urls:
        if url not in docs_urls:
            docs_urls.append(url)
            
    print(f"Total documentation URLs to check: {len(docs_urls)}")

    # 3. Connect to Qdrant
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        https=True
    )

    # 4. Audit URLs in Qdrant using Scroll
    present_urls = set()
    next_offset = None
    
    print("Auditing Qdrant collection...")
    while True:
        points, next_offset = client.scroll(
            collection_name=COLLECTION_NAME,
            with_payload=True,
            limit=100,
            offset=next_offset
        )
        for point in points:
            url = point.payload.get('url')
            if url:
                present_urls.add(url)
        
        if next_offset is None:
            break

    # 5. Analysis
    missing_urls = [url for url in docs_urls if url not in present_urls]
    found_urls = [url for url in docs_urls if url in present_urls]

    print("\n" + "="*50)
    print("INGESTION STATUS REPORT")
    print("="*50)
    print(f"Total Target URLs:   {len(docs_urls)}")
    print(f"URLs in Qdrant:      {len(found_urls)}")
    print(f"Missing URLs:        {len(missing_urls)}")
    print(f"Coverage:            {(len(found_urls)/len(docs_urls)*100):.2f}%")
    print("="*50)

    if found_urls:
        print("\nPRESENT URLs (Sample):")
        for url in found_urls[:10]:
            print(f"✅ {url}")
            
    if missing_urls:
        print("\nMISSING URLs (Sample):")
        for url in missing_urls[:10]:
            print(f"❌ {url}")

    print("\nTOTAL POINTS IN QDRANT:", client.count(COLLECTION_NAME).count)

if __name__ == "__main__":
    main()
