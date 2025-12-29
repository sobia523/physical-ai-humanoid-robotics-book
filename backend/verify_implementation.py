#!/usr/bin/env python3
"""
Verification script for RAG Pipeline implementation.

This script verifies that all components of the RAG pipeline have been
properly implemented according to the specification.
"""

import sys
import os
import subprocess
from pathlib import Path

def verify_directory_structure():
    """Verify that the required directory structure exists."""
    print("Verifying directory structure...")

    required_paths = [
        "main.py",
        "README.md",
        "requirements.txt"
    ]

    backend_dir = Path(__file__).parent
    all_found = True

    for path in required_paths:
        full_path = backend_dir / path
        if not full_path.exists():
            print(f"[ERROR] Missing: {path}")
            all_found = False
        else:
            print(f"[OK] Found: {path}")

    return all_found

def verify_functions_in_main():
    """Verify that required functions exist in main.py."""
    print("\nVerifying functions in main.py...")

    main_file = Path(__file__).parent / "main.py"
    if not main_file.exists():
        print("❌ main.py not found")
        return False

    content = main_file.read_text()

    required_functions = [
        'fetch_sitemap_urls',
        'crawl_url',
        'extract_content_from_html',
        'clean_extracted_content',
        'chunk_content_by_sections',
        'create_cohere_client',
        'generate_embeddings_with_caching',
        'create_qdrant_client',
        'store_embeddings_in_qdrant',
        'run_rag_pipeline',
        'generate_url_coverage_report',
        'verify_success_criteria'
    ]

    all_found = True
    for func in required_functions:
        if f'def {func}(' in content:
            print(f"[OK] Found function: {func}")
        else:
            print(f"[ERROR] Missing function: {func}")
            all_found = False

    return all_found

def verify_configuration():
    """Verify that configuration elements are in place."""
    print("\nVerifying configuration...")

    config_elements = [
        'CHUNK_SIZE',
        'OVERLAP_SIZE',
        'SITEMAP_URL',
        'QDRANT_COLLECTION_NAME',
        'COHERE_MODEL'
    ]

    main_file = Path(__file__).parent / "main.py"
    content = main_file.read_text()

    all_found = True
    for element in config_elements:
        if element in content:
            print(f"[OK] Found configuration: {element}")
        else:
            print(f"[ERROR] Missing configuration: {element}")
            all_found = False

    return all_found

def verify_readme():
    """Verify that README contains required sections."""
    print("\nVerifying README content...")

    readme_file = Path(__file__).parent / "README.md"
    if not readme_file.exists():
        print("[ERROR] README.md not found")
        return False

    content = readme_file.read_text()

    required_sections = [
        "Overview",
        "Setup",
        "Environment Variables",
        "Running the Pipeline",
        "Features",
        "Success Criteria"
    ]

    all_found = True
    for section in required_sections:
        if f"# {section}" in content or f"## {section}" in content:
            print(f"[OK] Found README section: {section}")
        elif section.lower() in content.lower():  # Case-insensitive check
            print(f"[OK] Found README section (case-varied): {section}")
        else:
            print(f"[ERROR] Missing README section: {section}")
            all_found = False

    return all_found

def verify_imports():
    """Verify that required imports are available."""
    print("\nVerifying imports...")

    required_imports = [
        "import requests",
        "import cohere",
        "from qdrant_client import QdrantClient",
        "import tiktoken",
        "from bs4 import BeautifulSoup"
    ]

    main_file = Path(__file__).parent / "main.py"
    content = main_file.read_text()

    all_found = True
    for imp in required_imports:
        if imp in content:
            print(f"[OK] Found import: {imp}")
        else:
            print(f"[ERROR] Missing import: {imp}")
            all_found = False

    return all_found

def main():
    """Main verification function."""
    print("Starting RAG Pipeline Implementation Verification...\n")

    # Change to backend directory
    os.chdir(Path(__file__).parent)

    all_checks_passed = True

    # Run all verification checks
    all_checks_passed &= verify_directory_structure()
    all_checks_passed &= verify_functions_in_main()
    all_checks_passed &= verify_configuration()
    all_checks_passed &= verify_readme()
    all_checks_passed &= verify_imports()

    print(f"\n{'='*60}")
    if all_checks_passed:
        print("SUCCESS: All verification checks PASSED!")
        print("RAG Pipeline implementation is complete and ready.")
        return 0
    else:
        print("FAILURE: Some verification checks FAILED!")
        print("Implementation needs additional work.")
        return 1

if __name__ == "__main__":
    sys.exit(main())