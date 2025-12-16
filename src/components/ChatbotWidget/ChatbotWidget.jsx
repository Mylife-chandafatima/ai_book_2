import React, { useState, useEffect, useRef } from 'react';
import './ChatbotWidget.css';

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [mode, setMode] = useState('book'); // 'book' or 'selection'
  const [selectedText, setSelectedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(() => {
    // Generate or retrieve session ID only on client side
    if (typeof window !== 'undefined' && window.localStorage) {
      const stored = localStorage.getItem('chatbot-session-id');
      if (stored) return stored;
      const newId = 'session-' + Date.now() + '-' + Math.random().toString(36).substr(2, 9);
      localStorage.setItem('chatbot-session-id', newId);
      return newId;
    }
    // Fallback for server-side rendering
    return 'session-' + Date.now() + '-' + Math.random().toString(36).substr(2, 9);
  });

  const messagesEndRef = useRef(null);
  const chatContainerRef = useRef(null);

  // Function to get selected text from the page
  const getSelectedText = () => {
    const selection = window.getSelection();
    return selection.toString().trim();
  };

  // Function to handle text selection
  const handleTextSelection = () => {
    const selected = getSelectedText();
    if (selected) {
      setSelectedText(selected);
      setMode('selection');
    }
  };

  // Add event listener for text selection
  useEffect(() => {
    const handleSelection = () => {
      // Only update selected text if in selection mode
      if (mode === 'selection') {
        const selected = getSelectedText();
        if (selected) {
          setSelectedText(selected);
        }
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, [mode]);

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Function to send message to backend
  const sendMessage = async () => {
    if (!input.trim() || isLoading) return;

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      text: input,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // Prepare the request based on mode
      let endpoint, requestData;

      if (mode === 'book') {
        endpoint = '/api/v1/chat/book';
        requestData = {
          question: input,
          session_id: sessionId
        };
      } else { // selection mode
        endpoint = '/api/v1/chat/selection';
        requestData = {
          question: input,
          selected_text: selectedText,
          session_id: sessionId
        };
      }

      // Make actual API call to backend
      // For now, use a fixed base URL to avoid process.env issues in browser
      const apiBaseUrl = '/api/v1';
      const response = await fetch(`${apiBaseUrl}${endpoint}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestData)
      });

      if (!response.ok) {
        throw new Error(`API call failed: ${response.status} ${response.statusText}`);
      }

      const response_data = await response.json();

      // Add bot response to chat
      const botMessage = {
        id: Date.now() + 1,
        text: response_data.answer || response_data.refusal_reason,
        sender: 'bot',
        timestamp: new Date(),
        citations: response_data.citations || [],
        wasRefused: response_data.was_refused || false
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date(),
        isError: true
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to handle Enter key press
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Function to render citations
  const renderCitations = (citations) => {
    if (!citations || citations.length === 0) return null;

    return (
      <div className="citations">
        <strong>Sources:</strong>
        <ul>
          {citations.map((citation, index) => (
            <li key={index}>
              <a
                href={citation.url}
                target="_blank"
                rel="noopener noreferrer"
                onClick={(e) => e.stopPropagation()}
              >
                {citation.module}, {citation.chapter}
              </a>
            </li>
          ))}
        </ul>
      </div>
    );
  };

  return (
    <div className="chatbot-widget">
      {isOpen ? (
        <div className="chatbot-container" ref={chatContainerRef}>
          <div className="chatbot-header">
            <div className="chatbot-title">Book Assistant</div>
            <div className="chatbot-controls">
              <select
                value={mode}
                onChange={(e) => setMode(e.target.value)}
                className="mode-selector"
                disabled={isLoading}
              >
                <option value="book">Book Mode</option>
                <option value="selection">Selection Mode</option>
              </select>
              <button
                className="chatbot-close"
                onClick={() => setIsOpen(false)}
                disabled={isLoading}
              >
                ×
              </button>
            </div>
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="chatbot-welcome">
                <p>Hello! I'm your book assistant.</p>
                <p>
                  <strong>Book Mode:</strong> Ask questions about the entire book
                </p>
                <p>
                  <strong>Selection Mode:</strong> Select text on the page, then ask questions about it
                </p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.sender}`}
                >
                  <div className="message-text">
                    {message.text}
                    {message.citations && renderCitations(message.citations)}
                    {message.isError && (
                      <div className="error-message">
                        An error occurred. Please check your connection and try again.
                      </div>
                    )}
                  </div>
                  <div className="message-timestamp">
                    {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="message bot">
                <div className="message-text">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chatbot-input-area">
            {mode === 'selection' && selectedText && (
              <div className="selected-text-preview">
                <small>Selected: "{selectedText.substring(0, 60)}{selectedText.length > 60 ? '...' : ''}"</small>
              </div>
            )}
            <div className="input-container">
              <textarea
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder={mode === 'book'
                  ? "Ask a question about the book..."
                  : "Ask a question about the selected text..."}
                className="chatbot-input"
                rows="1"
                disabled={isLoading}
              />
              <button
                onClick={sendMessage}
                className="chatbot-send-button"
                disabled={!input.trim() || isLoading}
              >
                Send
              </button>
            </div>
          </div>
        </div>
      ) : (
        <button
          className="chatbot-toggle-button"
          onClick={() => {
            setIsOpen(true);
            // Check if there's selected text when opening
            const selected = getSelectedText();
            if (selected) {
              setSelectedText(selected);
              setMode('selection');
            }
          }}
        >
          💬 Ask Book
        </button>
      )}
    </div>
  );
};

export default ChatbotWidget;