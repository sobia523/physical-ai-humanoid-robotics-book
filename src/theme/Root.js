import React from 'react';
import { ThemeProvider } from '../components/theme/ThemeProvider';

export default function Root({ children }) {
  return <ThemeProvider>{children}</ThemeProvider>;
}