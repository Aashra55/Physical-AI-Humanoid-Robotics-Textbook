import React, { useState, useEffect, useRef } from 'react';
import { MdDelete } from 'react-icons/md';
import styles from './styles.module.css';

// Chatbot icon (speech bubble)
const BotIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" className="feather feather-message-circle">
    <path d="M21 11.5a8.38 8.38 0 0 1-.9 3.8 8.5 8.5 0 0 1-7.6 4.7 8.38 8.38 0 0 1-3.8-.9L3 21l1.9-5.7a8.38 8.38 0 0 1-.9-3.8 8.5 8.5 0 0 1 7.6-4.7 8.38 8.38 0 0 1 3.8.9h.5a2 2 0 0 1 1.6 1.6v.5"></path>
  </svg>
);

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [showConfirm, setShowConfirm] = useState(false); // New state for dialog
  const chatEndRef = useRef(null);

  // 1. Load messages from localStorage on startup
  useEffect(() => {
    const savedMessages = localStorage.getItem('chatbot_history');
    if (savedMessages) {
      try {
        setMessages(JSON.parse(savedMessages));
      } catch (e) {
        console.error("Failed to parse chat history", e);
      }
    } else {
      setMessages([{ sender: 'bot', text: 'Hello! How can I help you with the content of this book?' }]);
    }
  }, []);

  // 2. Save messages to localStorage
  useEffect(() => {
    if (messages.length > 0) {
      localStorage.setItem('chatbot_history', JSON.stringify(messages));
    }
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    chatEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // 3. Clear Chat logic with custom dialog
  const handleClearChat = () => {
    localStorage.removeItem('chatbot_history');
    setMessages([{ sender: 'bot', text: 'Hello! How can I help you with the content of this book?' }]);
    setShowConfirm(false);
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!inputValue.trim()) return;

    const userMessage = { sender: 'user', text: inputValue };
    const newMessages = [...messages, userMessage];
    setMessages(newMessages);
    
    const query = inputValue;
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch('http://127.0.0.1:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ question: query }),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();
      const botMessage = { sender: 'bot', text: data.response || "I'm sorry, I couldn't process that." };
      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      console.error('An error occurred:', error);
      const errorMessage = { sender: 'bot', text: 'Sorry, I am having trouble connecting to the server.' };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      <button className={styles.chatButton} onClick={toggleChat}>
        <BotIcon />
      </button>

      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Custom Confirmation Dialog Overlay */}
          {showConfirm && (
            <div className={styles.modalOverlay}>
              <div className={styles.modal}>
                <h4>Clear History?</h4>
                <p>Are you sure you want to delete all messages? This cannot be undone.</p>
                <div className={styles.modalButtons}>
                  <button className={styles.confirmBtn} onClick={handleClearChat}>Delete</button>
                  <button className={styles.cancelBtn} onClick={() => setShowConfirm(false)}>Cancel</button>
                </div>
              </div>
            </div>
          )}

          <div className={styles.chatHeader}>
            <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
              <h3>RAG Chatbot</h3>
              <button 
                onClick={() => setShowConfirm(true)} 
                className={styles.clearButton}
                title="Delete all messages"
                style={{ 
                  background: 'none', 
                  border: 'none', 
                  color: '#ff4d4d', 
                  cursor: 'pointer', 
                  display: 'flex',
                  alignItems: 'center',
                  padding: '4px',
                  borderRadius: '4px',
                  transition: 'background 0.2s'
                }}
                onMouseOver={(e) => e.currentTarget.style.background = 'rgba(255, 77, 77, 0.1)'}
                onMouseOut={(e) => e.currentTarget.style.background = 'none'}
              >
                <MdDelete size={20} />
              </button>
            </div>
            <button onClick={toggleChat} className={styles.closeButton}>&times;</button>
          </div>
          <div className={styles.chatMessages}>
            {messages.map((msg, index) => (
              <div key={index} className={`${styles.message} ${styles[msg.sender]}`}>
                {msg.text}
              </div>
            ))}
            {isLoading && <div className={`${styles.message} ${styles.bot}`}>Thinking...</div>}
            <div ref={chatEndRef} />
          </div>
          <form onSubmit={handleSendMessage} className={styles.chatInputForm}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question..."
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading}>Send</button>
          </form>
        </div>
      )}
    </>
  );
};

export default Chatbot;
