# Data Model: Docusaurus UI Upgrade

## Entity: Navbar
- **Fields**:
  - title: string (text-only title "Physical AI & Humanoid Robotics Textbook")
  - position: string (sticky positioning)
  - border: string (subtle bottom border only)
  - typography: string (medium-bold, left-aligned academic typography)
  - searchEnabled: boolean (indicates if search bar is present)
  - searchPosition: string (right side of navbar)

## Entity: Module Card
- **Fields**:
  - id: string (unique identifier for the module)
  - moduleNumber: string (the module number)
  - title: string (bold module title)
  - description: string (academic description of the module)
  - path: string (route to the module content)
  - isClickable: boolean (entire card is clickable)
  - hoverEffect: string (subtle lift and border highlight)
  - ctaText: string ("Open Module" button text)
  - ctaAction: string (navigation to module path)

## Entity: Footer
- **Fields**:
  - isVisibleInLightMode: boolean (footer renders in light mode)
  - isVisibleInDarkMode: boolean (footer renders in dark mode)
  - title: string (book title)
  - copyright: string (copyright information)
  - githubLink: string (GitHub repository link)
  - styling: string (simple, minimal, professional academic styling)

## Entity: Search Functionality
- **Fields**:
  - indexingScope: array (modules, chapters, section headings)
  - inputField: boolean (search input field exists)
  - searchIcon: boolean (search icon present)
  - keyboardShortcut: string (Ctrl+K for search activation)
  - resultGrouping: string (results grouped by Module → Chapter → Section)
  - highlighting: boolean (keyword highlighting in results)
  - responseTime: number (search response time in milliseconds)

## Relationships:
- Navbar contains Search Functionality
- Homepage contains multiple Module Cards
- All pages contain Footer
- Search Functionality indexes content from Modules, Chapters, and Sections

## Validation Rules:
- Module Card path must be a valid route to existing content
- Navbar title must match specified text exactly
- Footer must render in both light and dark modes
- Search indexing scope must include all specified content types
- All navigation links must not result in "Page Not Found" errors