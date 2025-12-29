"""
Backend package initialization for RAG Chatbot Agent.

This module initializes the backend package and makes key modules available.
"""
from .config import Config

__all__ = [
    'Config',
    'agent',
    'utils'
]