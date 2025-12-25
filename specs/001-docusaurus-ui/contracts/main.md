# API Contracts: Professional Docusaurus UI System

## Overview

This document defines the API contracts for the Professional Docusaurus UI System. Since this is a frontend UI/UX redesign, the "APIs" primarily refer to component interfaces, theme system contracts, and data flow contracts within the React/Docusaurus application.

## Theme System Contracts

### Theme Provider Interface
```typescript
interface ThemeProviderProps {
  initialTheme?: 'light' | 'dark';
  children: React.ReactNode;
}

interface ThemeContextValue {
  theme: 'light' | 'dark';
  setTheme: (theme: 'light' | 'dark') => void;
  toggleTheme: () => void;
  isDarkMode: boolean;
}
```

**Contract Details**:
- The theme provider must be placed at the root of the application
- Initial theme defaults to 'light' if not specified
- `setTheme` must accept only 'light' or 'dark' values
- `toggleTheme` must switch between light and dark modes
- `isDarkMode` is a computed boolean based on current theme

### Theme CSS Variable Contract
```css
:root {
  /* Light mode variables */
  --ifm-color-primary: #2563EB;
  --ifm-color-primary-dark: #1D4ED8;
  --ifm-background-color: #FFFFFF;
  --ifm-background-surface-color: #F8FAFC;
  --ifm-text-color: #0F172A;
  --ifm-text-color-secondary: #475569;
  --ifm-border-color: #E2E8F0;
  --ifm-card-hover-background: #F1F5F9;
}

:root[data-theme='dark'] {
  /* Dark mode variables */
  --ifm-color-primary: #3B82F6;
  --ifm-color-primary-dark: #60A5FA;
  --ifm-background-color: #0B1120;
  --ifm-background-surface-color: #111827;
  --ifm-text-color: #E5E7EB;
  --ifm-text-color-secondary: #9CA3AF;
  --ifm-border-color: #1F2937;
  --ifm-card-hover-background: #1E293B;
}
```

## Component Contracts

### Homepage Hero Component
```typescript
interface HeroProps {
  title: string;
  subtitle: string;
  description: string;
  ctaText: string;
  ctaPath: string;
  onCtaClick?: () => void;
}

interface HeroContract {
  // Inputs
  title: "Physical AI & Humanoid Robotics" (required)
  subtitle: "Embodied Intelligence for the Real World" (required)
  description: Book focus description (required)
  ctaText: "Start Reading" (required)
  ctaPath: Path to Module 1 (required)

  // Outputs
  Must render H1 with title
  Must render H2 with subtitle
  Must render paragraph with description
  Must render button with CTA text linking to ctaPath
  Button must use accent color from theme
  Button hover must use accent hover color from theme
}
```

### Module Card Component
```typescript
interface ModuleCardProps {
  module: ModuleCardData;
  onClick?: (module: ModuleCardData) => void;
}

interface ModuleCardContract {
  // Inputs
  module: ModuleCardData with id, title, description (required)

  // Outputs
  Must display module title as heading
  Must display one-line description from existing content
  Must apply hover effect with background change and border highlight
  Must navigate to module index on click
  Must be responsive according to grid configuration
}
```

### Enhanced Search Component
```typescript
interface EnhancedSearchProps {
  config: SearchConfig;
  onSearch?: (query: SearchQuery) => void;
  onResultSelect?: (result: SearchResult) => void;
}

interface EnhancedSearchContract {
  // Inputs
  config: SearchConfig with scope and UX mode (required)

  // Outputs
  Must search across modules, chapters, sections, and content
  Must provide real-time results with keyword highlighting
  Must group results by module with clear hierarchy
  Must support keyboard shortcut (Ctrl+K/Cmd+K)
  Must show inline dropdown on desktop, full-screen overlay on mobile
  Results must follow Module → Chapter → Section hierarchy
}
```

### Navigation Bar Component
```typescript
interface CustomNavbarProps {
  config: NavbarConfig;
  activePath?: string;
}

interface CustomNavbarContract {
  // Inputs
  config: NavbarConfig with left/right items (required)
  activePath: Current page path for active link highlighting

  // Outputs
  Must be sticky
  Background must match page background
  Must have bottom border only (no shadow)
  Left side must show book title text
  Left side may show optional professional logo
  Right side must show documentation link and search bar
  Active links must be highlighted using accent color
}
```

## Data Flow Contracts

### Module Data Access Contract
```typescript
interface ModuleDataAccess {
  source: Docusaurus docs plugin metadata
  format: ModuleCardData[] array
  requirements:
    - Must preserve existing module structure (1-4)
    - Must extract one-line descriptions from existing content
    - Must maintain correct navigation paths
    - Must not modify or add modules
}
```

### Theme State Management Contract
```typescript
interface ThemeStateContract {
  persistence: Local storage with key 'theme-preference'
  initialization: Check system preference, then local storage, then default to light
  synchronization: All components must update immediately when theme changes
  accessibility: Must respect system preference by default
}
```

## Performance Contracts

### Page Load Time Contract
```typescript
interface PageLoadContract {
  maximum: 3 seconds
  measurement: From navigation start to DOMContentLoaded
  scope: All pages in the documentation site
  compliance: 95% of page loads must meet this requirement
}
```

### Search Response Time Contract
```typescript
interface SearchResponseContract {
  maximum: 1 second
  measurement: From first character input to results display
  scope: 95% of search queries
  compliance: 95% of queries must meet this requirement
}
```

### Theme Switching Contract
```typescript
interface ThemeSwitchContract {
  maximum: 100ms
  measurement: From toggle action to visual update completion
  scope: All theme switching actions
  compliance: Must be perceived as instantaneous
}
```

## Accessibility Contracts

### WCAG 2.1 AA Compliance Contract
```typescript
interface AccessibilityContract {
  contrast: Minimum 4.5:1 for normal text, 3:1 for large text
  keyboard: Full navigation support without mouse
  screenReader: Proper ARIA labels and semantic HTML
  testing: Automated accessibility testing in CI/CD
}
```

### Keyboard Navigation Contract
```typescript
interface KeyboardNavigationContract {
  tabOrder: Logical tab order following visual hierarchy
  focusIndicators: Visible focus indicators for all interactive elements
  skipLinks: Skip to main content link available
  shortcuts: Ctrl+K/Cmd+K for search functionality
}
```

## Responsive Design Contracts

### Breakpoint Contract
```typescript
interface ResponsiveContract {
  mobile: max-width: 768px
  tablet: 769px - 1024px
  desktop: min-width: 1025px

  moduleGrid:
    - Desktop: 2 columns
    - Tablet: 2 columns
    - Mobile: 1 column
}
```

## Error Handling Contracts

### Component Error Boundaries
```typescript
interface ErrorBoundaryContract {
  fallback: Graceful fallback UI for component failures
  logging: Errors must be logged for debugging
  recovery: Attempt to recover or provide alternative content
}
```

## Testing Contracts

### Component Testing Requirements
```typescript
interface ComponentTestingContract {
  unitTests: 90% code coverage for custom components
  integrationTests: End-to-end testing for theme switching
  accessibilityTests: Automated a11y testing for all components
  responsiveTests: Testing across all defined breakpoints
}
```

## Backwards Compatibility Contract

### Content Preservation
```typescript
interface BackwardsCompatibilityContract {
  content: No modifications to existing documentation content
  URLs: All existing URLs must continue to work
  navigation: All existing navigation paths must remain functional
  search: Existing search functionality must be enhanced, not replaced
}
```