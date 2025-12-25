---
id: 1
title: "Docusaurus UI Upgrade Specification"
stage: "spec"
date: "2025-12-24"
model: "Claude Sonnet 4.5"
feature: "docusaurus-ui-upgrade"
branch: "001-docusaurus-ui-upgrade"
user: "TLS"
command: "/sp.specify"
labels: ["ui", "docusaurus", "upgrade", "specification"]
links:
  spec: "../specs/001-docusaurus-ui-upgrade/spec.md"
  plan: null
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
  - "../specs/001-docusaurus-ui-upgrade/spec.md"
  - "../specs/001-docusaurus-ui-upgrade/checklists/requirements.md"
tests: []
---

# Prompt History Record: Docusaurus UI Upgrade Specification

## Summary

This PHR documents the creation of the specification for upgrading the Docusaurus UI to a modern, professional textbook-grade interface. The specification addresses navigation, layout, search, card sizing, footer visibility, and routing issues while preserving the existing color theme.

## Original Prompt

```
/sp.specify

Objective:
Upgrade the existing Docusaurus UI into a modern, professional, textbook-grade interface
without changing the already-applied color theme. Fix all navigation, layout, search,
card sizing, footer visibility, and routing issues.

Scope:
- UI refinement (NOT redesign from scratch)
- Navigation & routing fixes
- Navbar search bar implementation
- Module card professionalism upgrade
- Footer consistency (light & dark mode)
- Responsive behavior

----------------------------------------
1. Navbar Specification
----------------------------------------
- Remove logo completely.
- Replace with text-only title:
  "Physical AI & Humanoid Robotics Textbook"
- Title should be left-aligned, medium-bold, professional academic typography.
- Navbar must be sticky with subtle bottom border.

Search Bar (MANDATORY):
- Add a professional search bar to the navbar (right side).
- Use Docusaurus built-in search (Algolia or local search).
- Search must index:
  - Modules
  - Chapters
  - Section headings
- Search UX:
  - Input + search icon
  - Keyboard shortcut: Ctrl + K
  - Dropdown grouped results:
    Module → Chapter → Section
- Search must work in both light and dark modes.
- Search focus state should use existing theme accent color.

----------------------------------------
2. Module Cards (CRITICAL FIX)
----------------------------------------
Current Issue:
- Cards are too small
- Look non-professional
- Clicking shows "Page Not Found"

Fix Requirements:
- Increase card size significantly (enterprise-grade cards).
- Cards must include:
  - Module number
  - Module title (bold)
  - Short academic description
  - Professional CTA button ("Open Module")
- Entire card should be clickable (not only button).
- Hover effect:
  - Subtle lift
  - Border highlight
  - Smooth transition (no flashy animation)

Routing:
- Clicking card or button MUST open correct module route.
- Fix broken links causing "Page Not Found".
- Validate all sidebar + homepage routes.

----------------------------------------
3. Buttons
----------------------------------------
- Replace generic buttons with professional primary buttons.
- Button states:
  - Default
  - Hover
  - Active
  - Disabled
- Button color must match existing theme accent.
- Button click must correctly navigate to module pages.

----------------------------------------
4. Footer Fix
----------------------------------------
Current Issue:
- Footer visible only in dark mode
- Missing or broken in light mode

Fix:
- Footer must render in BOTH light and dark modes.
- Footer design:
  - Simple
  - Minimal
  - Professional academic style
- Footer content:
  - Book title
  - Copyright
  - GitHub link
- No heavy styling, no distraction.

----------------------------------------
5. Layout & Spacing
----------------------------------------
- Improve spacing between sections.
- Center content with max-width container.
- Ensure consistent margins and padding.
- Remove empty or broken sections.

----------------------------------------
6. Responsiveness
-------------------------
- Desktop: 2-column module grid.
- Tablet: responsive grid.
- Mobile:
  - Single-column cards
  - Search opens full-screen modal
- Navbar and footer must remain usable on all sizes.

----------------------------------------
7. Color Theme Handling
----------------------------------------
- Do NOT introduce a new theme.
- Keep existing color palette.
- Improve contrast, spacing, and hover usage.
- Ensure accessibility (readable text in both modes).

----------------------------------------
8. Quality Bar
----------------------------------------
- UI should resemble a professional AI textbook website.
- No broken links.
- No placeholder elements.
- No Page Not Found errors.
- Search must be functional.
```

## Response Summary

Successfully created a comprehensive specification for the Docusaurus UI upgrade feature with the following key elements:

### User Scenarios & Testing
- Created 4 prioritized user stories covering navigation, module cards, theme consistency, and responsive layout
- Each user story includes acceptance scenarios and independent test criteria
- Stories cover P1 (critical) and P2 (important) priorities

### Functional Requirements
- Defined 12 specific functional requirements (FR-001 to FR-012)
- Requirements cover navbar, search functionality, module cards, footer, layout, and responsive behavior
- All requirements are testable and unambiguous

### Success Criteria
- Established 7 measurable outcomes (SC-001 to SC-007)
- Criteria include performance metrics, success rates, and user experience measures
- All criteria are technology-agnostic and measurable

### Quality Validation
- Created a comprehensive quality checklist validating all specification requirements
- All checklist items marked as completed
- No [NEEDS CLARIFICATION] markers remain in the specification

## Outcome

The specification is complete and ready for the planning phase. It addresses all the requirements specified by the user:
- Professional navigation with search functionality
- Module card improvements with proper routing
- Consistent theme experience across light/dark modes
- Responsive layout improvements
- All while preserving the existing color theme

The specification is ready for `/sp.plan` to proceed with architectural planning.