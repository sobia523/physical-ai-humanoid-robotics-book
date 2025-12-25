---
id: 2
title: Update Constitution
stage: constitution
date: 2025-12-19
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: 001-digital-twin-module
user: TLS
command: /sp.constitution
labels: [constitution,ai-book,physical-ai,robotics,rag-chatbot,update]
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

Project: AI/Spec-Driven Book on Physical AI & Humanoid Robotics with Embedded RAG Chatbot

Core principles:
- Accuracy: All technical and scientific content must be verified against credible sources, including peer-reviewed articles, official documentation, and authoritative textbooks.
- Clarity: Content must be readable and understandable for a professional audience interested in AI, robotics, and software engineering.
- Reproducibility: All code snippets, workflows, and implementations must be traceable and replicable.
- Integration: The RAG chatbot must be seamlessly embedded in the published book and correctly retrieve answers from selected book content.
- Modern deployment: Book must follow best practices in Docusaurus deployment on GitHub Pages.

Key standards:
- All factual claims, algorithms, and code implementations must be traceable to reliable sources.
- Citation format: APA style.
- Minimum 50% of sources must be peer-reviewed or official documentation.
- Plagiarism tolerance: 0%.
- Writing clarity: Flesch-Kincaid grade 10–12 or equivalent technical readability.
- Code clarity: All code blocks must include comments explaining functionality.
- Chatbot behavior: Must only provide answers based on the selected book text and not hallucinate content.

Constraints:
- Book structure: Organized into Modules (minimum 4), each with 3–5 chapters.
- Source count: Minimum 20 sources.
- Deployment format: Docusaurus static site hosted on GitHub Pages with integrated chatbot functionality.
- Chatbot tech stack: OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, Qdrant Cloud Free Tier.

Success criteria:
- All book content verified and cited.
- Zero plagiarism.
- Deployed Docusaurus book accessible online.
- Fully functional RAG chatbot embedded, answering accurately from selected content.
- Code reproducible and ready-to-run.
```

## Response snapshot

Updated the project constitution to align with the new requirements. The updated constitution includes:

- Five core principles: Accuracy, Clarity, Reproducibility, Integration, and Modern Deployment
- Technical standards for citation (APA format), content quality (grade level 10-12), and code/chatbot standards
- Development workflow requirements for book structure (4+ modules, 3-5 chapters each), source standards (20+ sources), and deployment process (Docusaurus + GitHub Pages)
- Quality validation and testing standards for fact-checking, RAG chatbot validation, and code validation
- Governance section establishing the constitution as the governing document

The constitution was updated by revising the existing document to incorporate the specific requirements provided in the user input, including the exact wording of principles and constraints.

## Outcome

- ✅ Impact: Updated foundational governance document for the project
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with revised project constitution
- 🔁 Next prompts: N/A
- 🧠 Reflection: The updated constitution now precisely matches the project requirements and constraints specified by the user

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A