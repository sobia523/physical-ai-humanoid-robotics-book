"""
Test query phrases for RAG Pipeline Retrieval Validation.

This module contains various test queries designed to validate the similarity search functionality
of the Qdrant collection. These queries are designed to test different aspects of the retrieval system.
"""

# Test queries for ROS2 Nervous System content
ROS2_QUERIES = [
    "ROS2 fundamentals and basic concepts",
    "Python agents bridging in robotics",
    "URDF for humanoid robots",
    "Practical ROS2 integration techniques",
    "ROS2 communication patterns",
    "Robot Operating System 2 architecture",
    "ROS2 nodes and topics",
    "ROS2 services and actions",
    "ROS2 launch files",
    "ROS2 parameters and configuration"
]

# Test queries for Digital Twin Simulation content
DIGITAL_TWIN_QUERIES = [
    "Digital twin simulation for robotics",
    "Introduction to digital twins in robotics",
    "Simulation environments for robot development",
    "Digital twin architecture patterns",
    "Robot simulation frameworks",
    "Physics simulation for robots",
    "3D modeling for robot simulation",
    "Simulation accuracy and validation",
    "Digital twin data synchronization",
    "Robot simulation testing"
]

# General robotics and AI queries
GENERAL_QUERIES = [
    "Physical AI humanoid robotics",
    "Robotics fundamentals",
    "Humanoid robot design principles",
    "Robot control systems",
    "Robot sensing and perception",
    "Robot motion planning",
    "Robot learning algorithms",
    "Humanoid robot actuators",
    "Robot safety systems",
    "Robot ethics and safety"
]

# Combined test queries
TEST_QUERIES = ROS2_QUERIES + DIGITAL_TWIN_QUERIES + GENERAL_QUERIES

# Specific test queries for validation
VALIDATION_QUERIES = [
    "Physical AI Humanoid Robotics",
    "ROS2 nervous system fundamentals",
    "Digital twin simulation introduction",
    "Python agents bridging robotics",
    "URDF humanoid robot modeling",
    "Practical ROS2 integration"
]

def get_test_queries(category: str = "all") -> list:
    """
    Get test queries based on category.

    Args:
        category: Category of queries to return ('ros2', 'digital_twin', 'general', 'validation', 'all')

    Returns:
        List of test query strings
    """
    if category == "ros2":
        return ROS2_QUERIES
    elif category == "digital_twin":
        return DIGITAL_TWIN_QUERIES
    elif category == "general":
        return GENERAL_QUERIES
    elif category == "validation":
        return VALIDATION_QUERIES
    elif category == "all":
        return TEST_QUERIES
    else:
        raise ValueError(f"Unknown category: {category}. Use 'ros2', 'digital_twin', 'general', 'validation', or 'all'")


def get_sample_test_queries(limit: int = 5) -> list:
    """
    Get a sample of test queries for quick validation.

    Args:
        limit: Maximum number of queries to return

    Returns:
        List of sample test query strings
    """
    return VALIDATION_QUERIES[:limit]


if __name__ == "__main__":
    print(f"Total test queries available: {len(TEST_QUERIES)}")
    print(f"ROS2 queries: {len(ROS2_QUERIES)}")
    print(f"Digital twin queries: {len(DIGITAL_TWIN_QUERIES)}")
    print(f"General queries: {len(GENERAL_QUERIES)}")
    print(f"Validation queries: {len(VALIDATION_QUERIES)}")
    print("\nSample validation queries:")
    for i, query in enumerate(get_sample_test_queries(), 1):
        print(f"{i}. {query}")