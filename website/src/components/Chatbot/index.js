import React, { useState, useEffect, useRef } from 'react';
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
  const chatEndRef = useRef(null);

  const scrollToBottom = () => {
    chatEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      setMessages([{ sender: 'bot', text: 'Hello! How can I help you with the content of this book?' }]);
    }
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();
    console.log("1. handleSendMessage started.");
    if (!inputValue.trim()) {
      console.log("Input is empty, returning.");
      return;
    }

    const userMessage = { sender: 'user', text: inputValue };
    setMessages((prev) => [...prev, userMessage]);
    
    // Use a temporary variable for the query to avoid a race condition
    const query = inputValue;
    setInputValue('');
    
    console.log("2. Set isLoading to true.");
    setIsLoading(true);

    try {
      console.log("3. Attempting to fetch from backend at http://127.0.0.1:8000/chat...");
      const response = await fetch('http://127.0.0.1:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: query, selected_text: null }),
      });
      console.log("4. Fetch call completed. Response received:", response);

      if (!response.ok) {
        console.error("5a. Response was not OK. Status:", response.status);
        throw new Error(`API request failed with status ${response.status}`);
      }

      console.log("5b. Response is OK. Parsing JSON...");
      const data = await response.json();
      console.log("6. JSON parsed:", data);

      const botMessage = { sender: 'bot', text: data.response };
      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      console.error('7. CATCH BLOCK: An error occurred:', error);
      const errorMessage = { sender: 'bot', text: 'Sorry, I am having trouble connecting to the server.' };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      console.log("8. FINALLY BLOCK: Set isLoading to false.");
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
          <div className={styles.chatHeader}>
            <h3>RAG Chatbot</h3>
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
