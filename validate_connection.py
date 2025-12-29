"""
Validation script to test the agent connection and API setup.
"""
import sys
import os

# Add the project root to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_agent_import():
    """Test if the agent module can be imported successfully."""
    try:
        from backend.agent import agent_handler, initialize_agent
        print("[OK] Successfully imported agent module")
        return True
    except ImportError as e:
        print(f"[FAIL] Failed to import agent module: {e}")
        return False

def test_agent_initialization():
    """Test if the agent can be initialized."""
    try:
        from backend.agent import initialize_agent
        result = initialize_agent()
        if result.get("status") == "error":
            print(f"[FAIL] Agent initialization failed: {result.get('message')}")
            return False
        print(f"[OK] Agent initialized successfully")
        print(f"  Model: {result.get('model')}")
        print(f"  Collection: {result.get('qdrant_collection')}")
        return True
    except Exception as e:
        print(f"[FAIL] Error during agent initialization: {e}")
        return False

def test_config():
    """Test if configuration is properly loaded."""
    try:
        from backend.config import Config
        print("[OK] Configuration loaded successfully")
        print(f"  Qdrant URL: {Config.QDRANT_URL[:20]}..." if Config.QDRANT_URL else "  Qdrant URL: Not set")
        print(f"  Collection: {Config.QDRANT_COLLECTION_NAME}")
        print(f"  Top K: {Config.SIMILARITY_TOP_K}")
        return True
    except Exception as e:
        print(f"[FAIL] Error loading configuration: {e}")
        return False

def main():
    """Run all validation tests."""
    print("=" * 50)
    print("RAG Chatbot API Validation")
    print("=" * 50)
    print()

    tests = [
        ("Configuration", test_config),
        ("Agent Import", test_agent_import),
        ("Agent Initialization", test_agent_initialization),
    ]

    results = []
    for test_name, test_func in tests:
        print(f"\nTesting {test_name}...")
        print("-" * 50)
        result = test_func()
        results.append((test_name, result))

    print("\n" + "=" * 50)
    print("Summary")
    print("=" * 50)
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        symbol = "[OK]" if result else "[FAIL]"
        print(f"{symbol} {test_name}: {status}")

    all_passed = all(result for _, result in results)
    print("\n" + "=" * 50)
    if all_passed:
        print("[OK] All tests passed! API is ready to start.")
    else:
        print("[FAIL] Some tests failed. Please fix the issues before starting the API.")
    print("=" * 50)

    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())
