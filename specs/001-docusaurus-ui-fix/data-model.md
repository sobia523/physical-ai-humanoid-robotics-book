# Data Model: Docusaurus UI Fix

## Entity: Navbar
- **Fields**:
  - searchEnabled: boolean (indicates if search bar is present)
  - searchPosition: string (right side of navbar)
  - searchIndexing: array (modules, chapters, section headings)
  - keyboardShortcut: string (Ctrl+K for search activation)
  - themeCompatibility: boolean (works in light and dark modes)
  - responsiveBehavior: boolean (does not break responsiveness)

## Entity: Module Card
- **Fields**:
  - id: string (unique identifier for the module)
  - moduleNumber: string (the module number)
  - title: string (bold, academic-style title)
  - description: string (short description paragraph)
  - path: string (route to the module content)
  - isClickable: boolean (entire card is clickable)
  - hoverEffect: string (subtle elevation, accent border glow, smooth transition)
  - focusState: boolean (proper focus state for accessibility)
  - dimensions: object (width and height for professional appearance)
  - ctaText: string ("Open Module" button text)

## Entity: Footer
- **Fields**:
  - title: string (book title)
  - copyright: string (copyright information)
  - githubLink: string (optional GitHub link)
  - extraSectionsRemoved: boolean (all auto-generated sections removed)
  - themeConsistency: boolean (identical rendering in light and dark modes)
  - visibility: boolean (visible in both light and dark modes)

## Entity: Search Functionality
- **Fields**:
  - indexingScope: array (modules, chapters, section headings)
  - inputField: boolean (search input field exists)
  - searchIcon: boolean (search icon present)
  - keyboardShortcut: string (Ctrl+K for search activation)
  - focusHighlight: string (focus highlight using existing theme accent)
  - resultGrouping: string (results from modules, chapters, section headings)
  - mobileBehavior: string (search opens in modal on mobile)

## Relationships:
- Navbar contains Search Functionality
- Homepage contains multiple Module Cards
- All pages contain Footer
- Search Functionality indexes content from Modules, Chapters, and Sections

## Validation Rules:
- Module Card path must be a valid route to existing content
- Footer must render in both light and dark modes
- Search indexing scope must include all specified content types
- All navigation links must not result in "Page Not Found" errors
- Module Cards must appear professional and readable on laptop screens