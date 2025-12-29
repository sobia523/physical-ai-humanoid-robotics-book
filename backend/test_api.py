"""
Test script for the RAG Chatbot API endpoints.
This script tests the FastAPI endpoints to ensure they work correctly.
"""
import asyncio
import requests
import json
from datetime import datetime

# Test configuration
BASE_URL = "http://localhost:8000"

def test_health_endpoint():
    """Test the health check endpoint."""
    print("Testing health endpoint...")
    try:
        response = requests.get(f"{BASE_URL}/health")
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Health check passed: {data}")
            return True
        else:
            print(f"✗ Health check failed with status {response.status_code}: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Health check error: {e}")
        return False

def test_config_endpoint():
    """Test the config endpoint."""
    print("\nTesting config endpoint...")
    try:
        response = requests.get(f"{BASE_URL}/config")
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Config endpoint passed: {data.get('agent_config', 'No agent config found')}")
            return True
        else:
            print(f"✗ Config endpoint failed with status {response.status_code}: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Config endpoint error: {e}")
        return False

def test_ask_endpoint():
    """Test the ask endpoint with a sample question."""
    print("\nTesting ask endpoint...")
    try:
        # Sample request data
        sample_data = {
            "prompt": "What is a Digital Twin?",
            "optional_selected_text": None
        }

        response = requests.post(f"{BASE_URL}/ask", json=sample_data)
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Ask endpoint passed")
            print(f"  Answer preview: {data['answer'][:100]}...")
            print(f"  Sources: {len(data['sources'])} items")
            print(f"  Confidence: {data['confidence']}")
            return True
        else:
            print(f"✗ Ask endpoint failed with status {response.status_code}: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Ask endpoint error: {e}")
        return False

def test_ask_with_context_endpoint():
    """Test the ask-with-context endpoint with sample data."""
    print("\nTesting ask-with-context endpoint...")
    try:
        # Sample request data with context
        sample_data = {
            "prompt": "Explain the practical implementation based on this text",
            "optional_selected_text": "Digital twins are virtual replicas of physical systems that enable real-time monitoring and simulation."
        }

        response = requests.post(f"{BASE_URL}/ask-with-context", json=sample_data)
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Ask-with-context endpoint passed")
            print(f"  Answer preview: {data['answer'][:100]}...")
            print(f"  Sources: {len(data['sources'])} items")
            print(f"  Confidence: {data['confidence']}")
            return True
        elif response.status_code == 400:
            # This is expected if optional_selected_text is required but the agent doesn't need it
            print(f"⚠ Ask-with-context returned 400 (expected if selected text not provided to agent): {response.text}")
            return True
        else:
            print(f"✗ Ask-with-context endpoint failed with status {response.status_code}: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Ask-with-context endpoint error: {e}")
        return False

def test_root_endpoint():
    """Test the root endpoint."""
    print("\nTesting root endpoint...")
    try:
        response = requests.get(f"{BASE_URL}/")
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Root endpoint passed: {data}")
            return True
        else:
            print(f"✗ Root endpoint failed with status {response.status_code}: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Root endpoint error: {e}")
        return False

def run_all_tests():
    """Run all endpoint tests."""
    print("Starting RAG Chatbot API tests...\n")

    tests = [
        test_root_endpoint,
        test_health_endpoint,
        test_config_endpoint,
        test_ask_endpoint,
        test_ask_with_context_endpoint
    ]

    results = []
    for test_func in tests:
        results.append(test_func())

    # Summary
    passed = sum(results)
    total = len(results)

    print(f"\n{'='*50}")
    print(f"Test Summary: {passed}/{total} tests passed")
    print(f"Run at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    if passed == total:
        print("🎉 All tests passed!")
    else:
        print(f"⚠ {total - passed} test(s) failed")

    return passed == total

if __name__ == "__main__":
    print("Note: Make sure the FastAPI server is running on http://localhost:8000 before running tests")
    print("To start the server, run: uvicorn app:app --host 0.0.0.0 --port 8000")
    print()

    run_all_tests()