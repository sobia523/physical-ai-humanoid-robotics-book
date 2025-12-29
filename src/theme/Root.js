import React from 'react';
import { ThemeProvider } from '../components/theme/ThemeProvider';
import ChatbotSimple from '../components/Chatbot/ChatbotSimple';

export default function Root({ children }) {
  return (
    <>
      <ThemeProvider>
        {children}
      </ThemeProvider>
      <ChatbotSimple />
    </>
  );
}