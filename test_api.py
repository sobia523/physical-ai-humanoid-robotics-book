"""
Test script for RAG Chatbot FastAPI Backend

This script tests all the API endpoints to verify they are working correctly.
"""
import requests
import json
import time
from datetime import datetime

# API base URL
BASE_URL = "http://localhost:8000"

def print_section(title):
    """Print a section header."""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)

def print_result(test_name, success, details=""):
    """Print test result."""
    status = "[PASS]" if success else "[FAIL]"
    print(f"{status} {test_name}")
    if details:
        print(f"      {details}")

def test_root_endpoint():
    """Test the root endpoint."""
    print_section("Test 1: Root Endpoint")
    try:
        response = requests.get(f"{BASE_URL}/")
        success = response.status_code == 200
        print_result("Root endpoint responds", success, f"Status: {response.status_code}")

        if success:
            data = response.json()
            print(f"      Message: {data.get('message')}")
            print(f"      Timestamp: {data.get('timestamp')}")
        return success
    except Exception as e:
        print_result("Root endpoint responds", False, f"Error: {str(e)}")
        return False

def test_health_endpoint():
    """Test the health check endpoint."""
    print_section("Test 2: Health Check Endpoint")
    try:
        response = requests.get(f"{BASE_URL}/health")
        success = response.status_code == 200
        print_result("Health endpoint responds", success, f"Status: {response.status_code}")

        if success:
            data = response.json()
            print(f"      Status: {data.get('status')}")
            print(f"      Timestamp: {data.get('timestamp')}")
            if data.get('dependencies'):
                print(f"      Dependencies: {data.get('dependencies')}")
        return success
    except Exception as e:
        print_result("Health endpoint responds", False, f"Error: {str(e)}")
        return False

def test_config_endpoint():
    """Test the config endpoint."""
    print_section("Test 3: Config Endpoint")
    try:
        response = requests.get(f"{BASE_URL}/config")
        success = response.status_code == 200
        print_result("Config endpoint responds", success, f"Status: {response.status_code}")

        if success:
            data = response.json()
            print(f"      Agent Config: {json.dumps(data.get('agent_config'), indent=2)}")
        return success
    except Exception as e:
        print_result("Config endpoint responds", False, f"Error: {str(e)}")
        return False

def test_ask_endpoint():
    """Test the /ask endpoint with a sample query."""
    print_section("Test 4: Ask Endpoint (General Question)")
    try:
        query = "What is a Digital Twin?"
        payload = {"query": query}

        print(f"      Query: {query}")
        print(f"      Sending request...")

        start_time = time.time()
        response = requests.post(
            f"{BASE_URL}/ask",
            json=payload,
            headers={"Content-Type": "application/json"}
        )
        elapsed_time = time.time() - start_time

        success = response.status_code == 200
        print_result("Ask endpoint responds", success, f"Status: {response.status_code}, Time: {elapsed_time:.2f}s")

        if success:
            data = response.json()
            print(f"      Answer: {data.get('answer', '')[:200]}...")
            print(f"      Sources count: {len(data.get('sources', []))}")
            print(f"      Confidence: {data.get('confidence')}")
            print(f"      Timestamp: {data.get('timestamp')}")
        else:
            print(f"      Error: {response.text}")

        return success
    except Exception as e:
        print_result("Ask endpoint responds", False, f"Error: {str(e)}")
        return False

def test_ask_invalid_request():
    """Test the /ask endpoint with invalid request."""
    print_section("Test 5: Ask Endpoint (Invalid Request)")
    try:
        # Missing required 'query' field
        payload = {}

        response = requests.post(
            f"{BASE_URL}/ask",
            json=payload,
            headers={"Content-Type": "application/json"}
        )

        success = response.status_code == 422  # Unprocessable Entity
        print_result("Invalid request handled correctly", success, f"Status: {response.status_code}")

        if response.status_code == 422:
            print(f"      Validation error (expected): {response.json()}")

        return success
    except Exception as e:
        print_result("Invalid request handled correctly", False, f"Error: {str(e)}")
        return False

def test_ask_selected_endpoint():
    """Test the /ask-selected endpoint with selected text."""
    print_section("Test 6: Ask Selected Endpoint (Context-Aware)")
    try:
        query = "Explain this concept in detail"
        selected_text = "Digital twins are virtual representations of physical systems that can be used for simulation and analysis."
        payload = {
            "query": query,
            "selected_text": selected_text
        }

        print(f"      Query: {query}")
        print(f"      Selected Text: {selected_text[:100]}...")
        print(f"      Sending request...")

        start_time = time.time()
        response = requests.post(
            f"{BASE_URL}/ask-selected",
            json=payload,
            headers={"Content-Type": "application/json"}
        )
        elapsed_time = time.time() - start_time

        success = response.status_code == 200
        print_result("Ask-selected endpoint responds", success, f"Status: {response.status_code}, Time: {elapsed_time:.2f}s")

        if success:
            data = response.json()
            print(f"      Answer: {data.get('answer', '')[:200]}...")
            print(f"      Sources count: {len(data.get('sources', []))}")
            print(f"      Confidence: {data.get('confidence')}")
            print(f"      Context used: {data.get('context_used', '')[:100] if data.get('context_used') else 'None'}...")
            print(f"      Timestamp: {data.get('timestamp')}")
        else:
            print(f"      Error: {response.text}")

        return success
    except Exception as e:
        print_result("Ask-selected endpoint responds", False, f"Error: {str(e)}")
        return False

def test_ask_selected_missing_text():
    """Test the /ask-selected endpoint with missing selected_text."""
    print_section("Test 7: Ask Selected Endpoint (Missing Selected Text)")
    try:
        # Missing required 'selected_text' field
        payload = {"query": "Explain this"}

        response = requests.post(
            f"{BASE_URL}/ask-selected",
            json=payload,
            headers={"Content-Type": "application/json"}
        )

        success = response.status_code == 422  # Unprocessable Entity
        print_result("Missing selected_text handled correctly", success, f"Status: {response.status_code}")

        if response.status_code == 422:
            print(f"      Validation error (expected): {response.json()}")

        return success
    except Exception as e:
        print_result("Missing selected_text handled correctly", False, f"Error: {str(e)}")
        return False

def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("  RAG Chatbot API - Integration Tests")
    print("  Testing API at: " + BASE_URL)
    print("=" * 60)

    # Run all tests
    tests = [
        ("Root Endpoint", test_root_endpoint),
        ("Health Check", test_health_endpoint),
        ("Config Endpoint", test_config_endpoint),
        ("Ask Endpoint (Valid)", test_ask_endpoint),
        ("Ask Endpoint (Invalid)", test_ask_invalid_request),
        ("Ask Selected (Valid)", test_ask_selected_endpoint),
        ("Ask Selected (Invalid)", test_ask_selected_missing_text),
    ]

    results = []
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"[FAIL] {test_name}: Unexpected error: {str(e)}")
            results.append((test_name, False))

    # Print summary
    print_section("Test Summary")
    passed = sum(1 for _, result in results if result)
    total = len(results)

    for test_name, result in results:
        status = "[PASS]" if result else "[FAIL]"
        print(f"{status} {test_name}")

    print("\n" + "-" * 60)
    print(f"Results: {passed}/{total} tests passed ({passed/total*100:.1f}%)")
    print("-" * 60)

    if passed == total:
        print("\n[OK] All tests passed! API is working correctly.")
        return 0
    else:
        print(f"\n[FAIL] {total - passed} test(s) failed. Please check the API implementation.")
        return 1

if __name__ == "__main__":
    exit(main())
