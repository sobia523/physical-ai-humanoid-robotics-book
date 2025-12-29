import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def verify_ros2_ingestion():
    """
    Verify that all ROS2 chapter URLs have been ingested into Qdrant.
    """
    # Initialize Qdrant client
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        prefer_grpc=True
    )

    collection_name = "book_embeddings"

    # Target ROS2 URLs to verify
    target_urls = [
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/01/introduction",
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/02/ros2-fundamentals",
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/03/python-agents-bridging",
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/04/urdf-humanoids",
        "https://physical-ai-humanoid-robotics-book-rust.vercel.app/docs/modules/ros2-nervous-system/chapters/05/practical-integration"
    ]

    print("Verifying ROS2 Chapter URL Ingestion in Qdrant...")
    print("="*60)

    # Get total count of vectors in the collection
    try:
        collection_info = qdrant_client.get_collection(collection_name)
        total_vectors = collection_info.points_count
        print(f"Total vectors in collection '{collection_name}': {total_vectors}")
    except Exception as e:
        print(f"Error getting collection info: {e}")
        return

    # Check each target URL
    found_urls = []
    missing_urls = []

    for url in target_urls:
        try:
            # Search for points with this specific URL
            search_result = qdrant_client.scroll(
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

            # Check if we got results
            if isinstance(search_result, tuple):
                points = search_result[0]
            else:
                points = search_result

            if len(points) > 0:
                found_urls.append(url)
                print(f"[FOUND]: {url}")
                # Show how many chunks were stored for this URL
                all_points_result = qdrant_client.scroll(
                    collection_name=collection_name,
                    scroll_filter=models.Filter(
                        must=[
                            models.FieldCondition(
                                key="url",
                                match=models.MatchValue(value=url)
                            )
                        ]
                    )
                )

                if isinstance(all_points_result, tuple):
                    all_points = all_points_result[0]
                else:
                    all_points = all_points_result

                print(f"   -> {len(all_points)} chunks stored")
            else:
                missing_urls.append(url)
                print(f"[MISSING]: {url}")

        except Exception as e:
            print(f"[ERROR] checking {url}: {e}")
            missing_urls.append(url)

    print("\n" + "="*60)
    print("INGESTION VERIFICATION SUMMARY:")
    print(f"Total ROS2 URLs checked: {len(target_urls)}")
    print(f"Found in Qdrant: {len(found_urls)}")
    print(f"Missing from Qdrant: {len(missing_urls)}")
    print(f"Success Rate: {len(found_urls)/len(target_urls)*100:.2f}%")

    if missing_urls:
        print("\nMISSING URLS:")
        for url in missing_urls:
            print(f"  - {url}")

    if found_urls:
        print("\nINGESTED URLS:")
        for url in found_urls:
            print(f"  - {url}")

    # Also check for any ROS2-related content by searching for module name
    print("\n" + "="*60)
    print("ADDITIONAL: Checking for all ROS2 nervous system content...")

    try:
        ros2_points_result = qdrant_client.scroll(
            collection_name=collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="module",
                        match=models.MatchValue(value="ros2-nervous-system")
                    )
                ]
            )
        )

        if isinstance(ros2_points_result, tuple):
            ros2_points = ros2_points_result[0]
        else:
            ros2_points = ros2_points_result

        print(f"Total ROS2 nervous system chunks found: {len(ros2_points)}")

        # Get unique URLs with ROS2 content
        ros2_urls = set()
        for point in ros2_points:
            url = point.payload.get('url', '')
            if url:
                ros2_urls.add(url)

        print(f"Unique ROS2 nervous system URLs: {len(ros2_urls)}")

    except Exception as e:
        print(f"Error checking for ROS2 content: {e}")

if __name__ == "__main__":
    verify_ros2_ingestion()