# Data Model: Professional Docusaurus UI System

## Overview

This document defines the data structures and models required for implementing the Professional Docusaurus UI System. Since this is primarily a UI/UX redesign project with no changes to content structure, the data model focuses on theme state, UI configurations, and metadata needed for the enhanced UI components.

## Theme Data Model

### ThemeState
```typescript
interface ThemeState {
  currentTheme: 'light' | 'dark';
  themeConfig: ThemeConfig;
  isThemeSwitching: boolean;
}

interface ThemeConfig {
  // Light mode colors
  light: ThemeColors;
  // Dark mode colors
  dark: ThemeColors;
  // Animation settings
  transitionDuration: number; // in ms
}
```

### ThemeColors
```typescript
interface ThemeColors {
  // Background colors
  backgroundPrimary: string;      // #FFFFFF (light) / #0B1120 (dark)
  backgroundSecondary: string;    // #F8FAFC (light) / #111827 (dark)

  // Text colors
  textPrimary: string;            // #0F172A (light) / #E5E7EB (dark)
  textSecondary: string;          // #475569 (light) / #9CA3AF (dark)

  // Border colors
  borderColor: string;            // #E2E8F0 (light) / #1F2937 (dark)

  // Accent colors
  accentPrimary: string;          // #2563EB (light) / #3B82F6 (dark)
  accentHover: string;            // #1D4ED8 (light) / #60A5FA (dark)

  // Card colors
  cardHoverBackground: string;    // #F1F5F9 (light) / #1E293B (dark)

  // Shadow settings
  shadow: string;                 // soft shadows (light) / none (dark)
}
```

## Module Navigation Data Model

### ModuleCardData
```typescript
interface ModuleCardData {
  id: string;                     // Module identifier (e.g., "module-1")
  title: string;                  // Module name (e.g., "Module 1: Introduction")
  description: string;            // One-line description from existing content
  path: string;                   // Navigation path to module index
  index: number;                  // Module number (1-4)
  chaptersCount: number;          // Number of chapters in the module
  estimatedTime: string;          // Estimated reading time
}
```

### ModuleGridConfig
```typescript
interface ModuleGridConfig {
  desktopColumns: number;         // 2 columns for desktop
  tabletColumns: number;          // 2 columns for tablet
  mobileColumns: number;          // 1 column for mobile
  gapSize: string;                // Spacing between cards
  maxCardWidth: string;           // Maximum width of individual cards
}
```

## Search Enhancement Data Model

### SearchConfig
```typescript
interface SearchConfig {
  scope: SearchScope;
  uxMode: 'inline' | 'overlay';   // Desktop vs mobile search UX
  keyboardShortcut: string;       // 'Ctrl+K' or 'Cmd+K'
  resultGrouping: 'module' | 'section';
  maxResults: number;
  debounceTime: number;           // ms for real-time search
}

interface SearchScope {
  includeModules: boolean;
  includeChapters: boolean;
  includeSections: boolean;
  includeContent: boolean;
}

interface SearchQuery {
  term: string;
  filters: SearchFilters;
  timestamp: Date;
}

interface SearchFilters {
  module?: string;
  contentType?: 'module' | 'chapter' | 'section';
  dateRange?: { start: Date; end: Date };
}

interface SearchResult {
  id: string;
  title: string;
  contentPreview: string;
  hierarchy: SearchResultHierarchy;
  relevanceScore: number;
  path: string;
}

interface SearchResultHierarchy {
  module: string;
  chapter?: string;
  section?: string;
}
```

## Navigation Data Model

### NavbarConfig
```typescript
interface NavbarConfig {
  isSticky: boolean;
  backgroundColor: string;        // Matches page background
  hasBottomBorder: boolean;
  hasShadow: boolean;
  leftItems: NavbarItem[];
  rightItems: NavbarItem[];
}

interface NavbarItem {
  type: 'title' | 'logo' | 'link' | 'search';
  content: string | JSX.Element;
  isActive?: boolean;
  path?: string;
  hidden?: boolean;               // For optional logo
}
```

### NavigationState
```typescript
interface NavigationState {
  activePage: string;
  breadcrumbs: Breadcrumb[];
  previousPage?: string;
  nextPage?: string;
}

interface Breadcrumb {
  title: string;
  path: string;
  isCurrent: boolean;
}
```

## Typography Data Model

### TypographyConfig
```typescript
interface TypographyConfig {
  fontFamily: {
    primary: string;
    code: string;
  };
  headingScale: HeadingScale;
  bodyScale: BodyScale;
  lineHeight: {
    heading: number;
    body: number;
    code: number;
  };
  fontWeight: {
    regular: number;
    medium: number;
    bold: number;
  };
}

interface HeadingScale {
  h1: string;                     // Book title
  h2: string;                     // Module titles
  h3: string;                     // Chapter titles
  h4: string;                     // Section titles
}

interface BodyScale {
  large: string;                  // Lead paragraphs
  regular: string;                // Body text
  small: string;                  // Captions, metadata
  xSmall: string;                 // Footnotes, fine print
}
```

## UI Component State Models

### HomePageData
```typescript
interface HomePageData {
  hero: HeroSectionData;
  modules: ModuleCardData[];
  cta: CTAButtonData;
}

interface HeroSectionData {
  title: string;                  // "Physical AI & Humanoid Robotics"
  subtitle: string;               // "Embodied Intelligence for the Real World"
  description: string;            // Book focus description
  ctaButton: CTAButtonData;
}

interface CTAButtonData {
  label: string;                  // "Start Reading"
  path: string;                   // Link to Module 1
  accentColor: string;            // Theme accent color
  hoverColor: string;             // Theme accent hover color
  isActive: boolean;
}
```

### FooterConfig
```typescript
interface FooterConfig {
  content: FooterContent;
  styling: FooterStyling;
}

interface FooterContent {
  copyrightText: string;
  additionalLinks: boolean;       // Should be false per spec
}

interface FooterStyling {
  backgroundColor: string;        // Neutral background
  textColor: string;
  hasBorder: boolean;
}
```

## Responsive Design Data Model

### Breakpoints
```typescript
interface Breakpoints {
  mobile: number;                 // 320px - 768px
  tablet: number;                 // 769px - 1024px
  desktop: number;                // 1025px+
  desktopLarge: number;           // 1440px+
}

interface ResponsiveConfig {
  breakpoints: Breakpoints;
  componentOverrides: {
    moduleGrid: {
      [key in keyof Breakpoints]?: ModuleGridConfig;
    };
    searchUX: {
      [key in keyof Breakpoints]?: 'inline' | 'overlay';
    };
    typography: {
      [key in keyof Breakpoints]?: TypographyAdjustments;
    };
  };
}

interface TypographyAdjustments {
  headingScale?: Partial<HeadingScale>;
  bodyScale?: Partial<BodyScale>;
  lineHeight?: Partial<{ heading: number; body: number }>;
}
```

## Performance Data Model

### PerformanceMetrics
```typescript
interface PerformanceMetrics {
  pageLoadTime: {
    target: number;               // < 3 seconds
    warningThreshold: number;     // 2.5 seconds
    maxAllowed: number;           // 5 seconds
  };
  searchResponseTime: {
    target: number;               // < 1 second
    warningThreshold: number;     // 0.8 seconds
    maxAllowed: number;           // 2 seconds
  };
  themeSwitchTime: {
    target: number;               // < 100ms (instantaneous)
    maxAllowed: number;           // 200ms
  };
}

interface BundleSizeMetrics {
  maxJSBundle: number;            // in KB
  maxCSSBundle: number;           // in KB
  maxImageSize: number;           // in KB per image
  totalPageWeight: number;        // in KB
}
```

## Accessibility Data Model

### AccessibilityConfig
```typescript
interface AccessibilityConfig {
  wcagLevel: 'AA';                // WCAG 2.1 AA compliance
  contrastRatios: {
    normalText: number;           // Minimum 4.5:1
    largeText: number;            // Minimum 3:1
    graphics: number;             // Minimum 3:1
  };
  keyboardNavigation: {
    enabled: boolean;
    focusIndicators: boolean;
    skipLinks: boolean;
  };
  screenReader: {
    ariaLabels: boolean;
    semanticHTML: boolean;
    alternativeText: boolean;
  };
}
```

## Theme Context Model

### ThemeContext
```typescript
interface ThemeContextType {
  theme: ThemeState;
  setTheme: (theme: 'light' | 'dark') => void;
  toggleTheme: () => void;
  applyTheme: (theme: 'light' | 'dark') => void;
  isDarkMode: boolean;
  themeColors: ThemeColors;
}
```

## Validation Schema

All data models must adhere to the following validation rules:

1. **Theme Colors**: Must match exact hex values specified in the feature specification
2. **Module Data**: Must preserve existing content structure and paths
3. **Accessibility**: Must meet WCAG 2.1 AA compliance requirements
4. **Performance**: Must meet defined performance metrics
5. **Responsive**: Must work across all specified breakpoints
6. **No Content Modification**: Data models must not alter existing documentation content