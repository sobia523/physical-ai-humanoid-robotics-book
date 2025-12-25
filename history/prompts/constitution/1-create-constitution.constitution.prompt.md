---
id: 1
title: Create Constitution
stage: constitution
date: 2025-12-19
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: 001-digital-twin-module
user: TLS
command: /sp.constitution
labels: [constitution,ai-book,physical-ai,robotics,rag-chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
/sp.constitution

Project: AI/Spec-Driven Book on Physical AI & Humanoid Robotics with Integrated RAG Chatbot

---

Core Principles:
1. **Accuracy & Reliability**
   - All technical content, algorithms, AI/robotics implementations, and design decisions must be validated against authoritative sources.
   - Peer-reviewed journals, official SDK/documentation references, and textbooks should be prioritized.
   - Claims must be traceable to cited sources.

2. **Clarity & Accessibility**
   - Content should be readable and understandable for a professional audience in AI, robotics, and software engineering.
   - Avoid unnecessary jargon; provide explanations and examples for complex concepts.
   - Maintain consistent terminology throughout modules and chapters.

3. **Reproducibility & Code Integrity**
   - All code snippets, workflows, and RAG chatbot integrations must be fully reproducible.
   - Code must include explanatory comments and be tested to run without errors in the specified environment.
   - Include step-by-step setup instructions for both book content and chatbot deployment.

4. **Seamless Integration**
   - The RAG chatbot must be embedded in the book in a way that allows users to query only the selected content.
   - The chatbot should leverage the specified tech stack (OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, Qdrant Cloud Free Tier) with proper documentation for each component.
   - Ensure responsive design and compatibility with deployed GitHub Pages.

5. **Modern Deployment & Maintenance**
   - Follow Docusaurus best practices for static site generation and deployment.
   - Ensure site structure supports modular expansion (future updates and new chapters/modules).

---

Key Standards:
1. **Citation & Source Management**
   - Citation format: APA style.
   - Minimum 50% of sources must be peer-reviewed or official documentation.
   - Plagiarism tolerance: 0%; all content must pass plagiarism checks before final submission.

2. **Content Quality**
   - Flesch-Kincaid grade level: 10–12 (technical readability suitable for professional audience).
   - Visuals, diagrams, and tables should enhance understanding of AI/robotics concepts.
   - Use consistent formatting and typography across all modules.

3. **Code & Chatbot Standards**
   - Code snippets must be commented and ready-to-run.
   - RAG chatbot responses must strictly reference the book content; hallucination is unacceptable.
   - Include automated unit tests or validation scripts where possible for code examples.

---

Constraints:
- **Book Structure**
  - Organized into Modules (minimum 4), each with 3–5 chapters.
  - Chapters should include theory, examples, and practical code demonstrations.
  - Each module should have a "RAG Integration" section explaining related chatbot functionality.

- **Source & Reference**
  - Minimum 20 credible sources.
  - Peer-reviewed articles, official documentation, and authoritative textbooks prioritized.

- **Deployment**
  - Use Docusaurus for static site generation.
  - Deploy on GitHub Pages.
  - Ensure chatbot integration works post-deployment on the live site.

- **Technical Stack**
  - RAG Chatbot: OpenAI Agents/ChatKit SDKs, FastAPI backend, Neon Serverless Postgres, Qdrant Cloud Free Tier.
  - All software dependencies and environment configurations must be documented.

---

Decisions & Documentation:
- Maintain a **decision log** to record all design and technical choices:
  - Book structure (modules/chapters)
  - AI/robotics algorithms included
  - RAG chatbot architecture and tech stack choices
  - Deployment strategies and tools
- Document **options considered**, **trade-offs**, and **justifications** for each major decision.

---

Quality Validation & Testing:
- Verify all claims against sources; maintain a **fact-checking checklist**.
- Test RAG chatbot responses for:
  - Accuracy (answers correctly based on selected content)
  - Relevance (ignores unrelated content)
  - Robustness (handles edge cases and unexpected input)
- Validate code snippets by running them in the speci
```

## Response snapshot

Created a comprehensive constitution for the AI/Spec-Driven Book on Physical AI & Humanoid Robotics with Integrated RAG Chatbot project. The constitution includes:

- Six core principles covering accuracy, clarity, reproducibility, seamless integration, modern deployment, and authoritative source mandate
- Technical standards for citation, content quality, and code/chatbot standards
- Development workflow requirements for book structure, sources, deployment, and decision documentation
- Quality validation and testing standards for fact-checking, RAG chatbot validation, and code validation
- Governance section establishing the constitution as the governing document

The constitution was created by taking the provided user requirements and structuring them into the template format, replacing all placeholder tokens with specific content relevant to the project.

## Outcome

- ✅ Impact: Created foundational governance document for the project
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with comprehensive project constitution
- 🔁 Next prompts: N/A
- 🧠 Reflection: The constitution establishes clear principles and standards that will guide the development of the AI/Spec-Driven Book project

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A