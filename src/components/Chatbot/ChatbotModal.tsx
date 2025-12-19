import React, { useState, useEffect, useRef } from 'react';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
}

interface ChatbotModalProps {
  messages: Message[];
  onSendMessage: (message: string) => void;
  isLoading: boolean;
  colorMode: 'light' | 'dark';
  onClose: () => void;
}

export const ChatbotModal: React.FC<ChatbotModalProps> = ({ 
  messages, 
  onSendMessage, 
  isLoading, 
  colorMode,
  onClose
}) => {
  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (inputValue.trim() && !isLoading) {
      onSendMessage(inputValue);
      setInputValue('');
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e as any);
    }
  };

  return (
    <div className={`chatbot-modal ${colorMode}`}>
      <div className="chatbot-header">
        <h3>AI Assistant</h3>
        <button 
          className="chatbot-close-button"
          onClick={onClose}
          aria-label="Close chat"
        >
          ×
        </button>
      </div>
      
      <div className="chatbot-messages">
        {messages.length === 0 ? (
          <div className="chatbot-welcome">
            <p>Hello! I'm your AI assistant for the Physical AI & Robotics book.</p>
            <p>You can ask questions about the content or select text and ask questions about it.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div 
              key={message.id} 
              className={`chatbot-message ${message.role}`}
            >
              <div className="chatbot-message-content">
                {message.content}
              </div>
              <div className="chatbot-message-timestamp">
                {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="chatbot-message assistant">
            <div className="chatbot-message-content">
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
      
      <form className="chatbot-input-form" onSubmit={handleSubmit}>
        <textarea
          ref={inputRef}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about the book content..."
          disabled={isLoading}
          rows={1}
          className="chatbot-input"
        />
        <button 
          type="submit" 
          disabled={!inputValue.trim() || isLoading}
          className="chatbot-send-button"
        >
          <svg 
            xmlns="http://www.w3.org/2000/svg" 
            width="16" 
            height="16" 
            viewBox="0 0 24 24" 
            fill="none" 
            stroke="currentColor" 
            strokeWidth="2" 
            strokeLinecap="round" 
            strokeLinejoin="round"
          >
            <line x1="22" y1="2" x2="11" y2="13"></line>
            <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
          </svg>
        </button>
      </form>
    </div>
  );
};