import React, { useState, useRef, useEffect } from 'react';
import styles from './Chatbot.module.css';

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Log component mount
  useEffect(() => {
    console.log('✅ Chatbot component mounted successfully');
    console.log('📡 API URL:', API_URL);
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Listen for text selection
  useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();
      if (text && text.length > 0 && text.length < 5000) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
    };
  }, []);

  const toggleChatbot = () => {
    console.log('🖱️ Chatbot button clicked! Current state:', isOpen);
    setIsOpen(!isOpen);
    console.log('📱 Chatbot will be:', !isOpen ? 'OPEN' : 'CLOSED');
  };

  const sendMessage = async (e) => {
    e.preventDefault();
    if (!inputValue.trim()) return;

    const userMessage = inputValue.trim();
    setInputValue('');

    // Add user message to chat
    setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
    setIsLoading(true);

    try {
      // Determine which endpoint to use
      const endpoint = selectedText ? '/ask-selected' : '/ask';
      const requestBody = selectedText
        ? { query: userMessage, selected_text: selectedText }
        : { query: userMessage };

      const response = await fetch(`${API_URL}${endpoint}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add bot response to chat
      setMessages(prev => [
        ...prev,
        {
          role: 'assistant',
          content: data.answer,
          sources: data.sources,
          confidence: data.confidence,
        },
      ]);

      // Clear selected text after using it
      if (selectedText) {
        setSelectedText('');
      }
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages(prev => [
        ...prev,
        {
          role: 'assistant',
          content: 'Sorry, I encountered an error while processing your request. Please make sure the API server is running.',
          error: true,
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const clearSelectedText = () => {
    setSelectedText('');
  };

  return (
    <>
      {/* Chat Toggle Button */}
      <button
        className={`${styles.chatToggle} ${isOpen ? styles.chatToggleActive : ''}`}
        onClick={toggleChatbot}
        aria-label="Toggle chatbot"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>RAG Chatbot</h3>
            <button onClick={toggleChatbot} className={styles.closeButton}>
              ✕
            </button>
          </div>

          {selectedText && (
            <div className={styles.selectedTextIndicator}>
              <span className={styles.selectedTextLabel}>
                Selected text will be used as context:
              </span>
              <div className={styles.selectedTextContent}>
                {selectedText.substring(0, 100)}
                {selectedText.length > 100 ? '...' : ''}
              </div>
              <button onClick={clearSelectedText} className={styles.clearButton}>
                Clear
              </button>
            </div>
          )}

          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>
                  👋 Hi! I'm your RAG Chatbot assistant. I can answer questions
                  about the book content.
                </p>
                <p>
                  <strong>Tip:</strong> Select any text on the page, then ask a
                  question about it!
                </p>
              </div>
            )}

            {messages.map((message, index) => (
              <div
                key={index}
                className={`${styles.message} ${
                  message.role === 'user'
                    ? styles.userMessage
                    : styles.assistantMessage
                } ${message.error ? styles.errorMessage : ''}`}
              >
                <div className={styles.messageContent}>{message.content}</div>
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <strong>Sources:</strong>
                    <ul>
                      {message.sources.map((source, idx) => (
                        <li key={idx}>
                          <a href={source.url} target="_blank" rel="noopener noreferrer">
                            {source.title || source.heading}
                          </a>
                          {source.relevance_score && (
                            <span className={styles.relevanceScore}>
                              ({(source.relevance_score * 100).toFixed(0)}% relevant)
                            </span>
                          )}
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.loadingIndicator}>
                  <span>●</span>
                  <span>●</span>
                  <span>●</span>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={sendMessage} className={styles.chatInputForm}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question about the book..."
              className={styles.chatInput}
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={isLoading || !inputValue.trim()}
            >
              {isLoading ? '...' : '➤'}
            </button>
          </form>
        </div>
      )}
    </>
  );
};

export default Chatbot;
