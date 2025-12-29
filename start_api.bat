@echo off
REM Script to start the RAG Chatbot API server

echo Starting RAG Chatbot FastAPI Backend...
echo.

REM Set the environment if not already set
set PYTHONPATH=%PYTHONPATH%;%CD%

REM Check if .env file exists
if not exist .env (
    echo WARNING: .env file not found!
    echo Please create a .env file with required environment variables.
    echo.
)

REM Start the FastAPI server
echo Starting uvicorn server on http://0.0.0.0:8000
echo Press Ctrl+C to stop the server
echo.

python -m uvicorn backend.api:app --host 0.0.0.0 --port 8000 --reload