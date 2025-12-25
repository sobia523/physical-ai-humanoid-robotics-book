---
id: 2
title: "Docusaurus UI System Spec"
stage: "spec"
date: "2025-12-23"
model: "Claude Sonnet 4.5"
feature: "docusaurus-ui"
branch: "001-docusaurus-ui"
user: "TLS"
command: "/sp.specify"
labels: ["ui", "docusaurus", "theme", "specification"]
links:
  spec: "../specs/001-docusaurus-ui/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "../specs/001-docusaurus-ui/spec.md"
  - "../specs/001-docusaurus-ui/checklists/requirements.md"
tests: []
---

# Prompt History Record: Docusaurus UI System Specification

## Summary

This PHR documents the creation of the specification for the Professional Docusaurus UI System for the Physical AI & Humanoid Robotics textbook. The specification outlines requirements for a modern, academic, and professional UI/UX redesign while preserving all existing content.

## Original Prompt

```
/sp.specify

Project: Physical AI & Humanoid Robotics — Professional Docusaurus UI System

Target audience:
- AI & Robotics students
- Engineers and researchers
- Readers of advanced technical textbooks

Primary objective:
Deliver a modern, academic, and professional UI/UX for an existing Docusaurus-based book, without altering, deleting, or restructuring any existing modules, chapters, or content.

ABSOLUTE CONSTRAINTS:
- Do NOT delete or modify existing modules or chapters
- Do NOT add new modules, pages, or features
- Do NOT rewrite documentation content
- UI, theme, layout, and navigation ONLY

--------------------------------------------------
COLOR THEME SYSTEM (MANDATORY)
--------------------------------------------------

Light Mode Theme:
- Background (primary): #FFFFFF
- Background (secondary sections/cards): #F8FAFC
- Text (primary): #0F172A
- Text (secondary): #475569
- Border color: #E2E8F0
- Accent / Primary action: #2563EB
- Accent hover: #1D4ED8
- Card hover background: #F1F5F9
- Shadow: soft, low opacity, used sparingly

Dark Mode Theme:
- Background (primary): #0B1120
- Background (secondary sections/cards): #111827
- Text (primary): #E5E7EB
- Text (secondary): #9CA3AF
- Border color: #1F2937
- Accent / Primary action: #3B82F6
- Accent hover: #60A5FA
- Card hover background: #1E293B
- Shadow: removed; borders used instead

Theme rules:
- Only ONE accent color
- No gradients
- No bright or decorative colors
- Identical spacing and hierarchy in light and dark mode

--------------------------------------------------
HOMEPAGE (FRONT PAGE)
--------------------------------------------------

Hero Section:
- H1 (Main Heading):
  "Physical AI & Humanoid Robotics"
- Subtitle (H2):
  "Embodied Intelligence for the Real World"
- Supporting paragraph (2–3 lines):
  Describe the book's focus on Physical AI, humanoid systems, ROS 2, simulation, perception, planning, and real-world deployment.
- Primary CTA button:
  Label: "Start Reading"
  Color: Accent color
  Hover: Accent hover color
  Action: Navigate to existing Module 1 route

--------------------------------------------------
MODULE NAVIGATION (CARDS)
--------------------------------------------------

- Display existing modules (Module 1–4) as cards
- No new modules added
- Card design:
  - Title (Module name)
  - One-line description from existing content
- Card behavior:
  - Hover: subtle background change + border highlight
  - Click: opens respective module index
- Responsive grid:
  - Desktop: 2 columns
  - Tablet: 2 columns
  - Mobile: 1 column

--------------------------------------------------
NAVBAR
--------------------------------------------------

- Sticky navbar
- Background matches page background
- Bottom border only (no shadow)
- Left:
  - Book title text
  - Logo OPTIONAL; hide if not professional
- Right:
  - Documentation link
  - Search bar
- Active links highlighted using accent color

--------------------------------------------------
SEARCH BAR (PROFESSIONAL REQUIREMENTS)
--------------------------------------------------

Search scope:
- Modules
- Chapters
- Section headings inside chapters
- Documentation content ONLY

Search UX:
- Desktop:
  - Inline dropdown below search input
- Mobile:
  - Full-screen search overlay

Search features:
- Real-time results
- Highlight matched keywords
- Group results by Module
- Keyboard shortcut support (Ctrl+K / Cmd+K)
- Clear hierarchy:
  Module → Chapter → Section

Visual style:
- Background matches theme
- Border-only focus state
- No heavy animations

--------------------------------------------------
TYPOGRAPHY
--------------------------------------------------

- Clean technical documentation fonts
- Clear hierarchy:
  - H1: book title
  - H2: module titles
  - H3/H4: chapter and sections
- Comfortable reading line-height
- No decorative fonts

--------------------------------------------------
FOOTER
--------------------------------------------------

- Minimal footer
- Neutral background
- Copyright text only
- No extra links or pages

-------------------------------------------
SUCCESS CRITERIA
--------------------------------------------------

- UI looks modern, academic, and professional
- Fully usable in light and dark mode
- Search works across modules and chapters
- Navigation is intuitive
- Existing book content remains untouched
```

## Response Summary

Created comprehensive specification for the Professional Docusaurus UI System including:

1. Detailed functional requirements for:
   - Homepage redesign with hero section
   - Module navigation cards
   - Navigation bar improvements
   - Search bar enhancements
   - Typography improvements
   - Footer redesign

2. Complete color theme system implementation:
   - Light mode specifications
   - Dark mode specifications
   - Theme consistency requirements

3. User scenarios and testing criteria
4. Non-functional requirements for performance and accessibility
5. Success criteria with quantitative and qualitative measures

## Outcome

The specification document is complete and ready for the planning phase. All requirements are clearly defined, testable, and focused on the user experience without including implementation details. The specification adheres to the constraints of preserving existing content while improving the UI/UX.