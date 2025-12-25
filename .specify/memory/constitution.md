<!-- Sync Impact Report:
Version change: 1.0.0 → 1.1.0 (principle updates and constraint additions)
Modified principles: Accuracy & Reliability → Accuracy, Clarity & Accessibility → Clarity, Reproducibility & Code Integrity → Reproducibility, Seamless Integration → Integration, Modern Deployment & Maintenance → Modern Deployment
Added sections: Core Principles (5), Technical Standards, Development Workflow, Quality Validation & Testing, Governance
Templates requiring updates: N/A (constitution updated)
Follow-up TODOs: None
-->
# AI/Spec-Driven Book on Physical AI & Humanoid Robotics with Embedded RAG Chatbot Constitution

## Core Principles

### Accuracy
All technical and scientific content must be verified against credible sources, including peer-reviewed articles, official documentation, and authoritative textbooks. All factual claims, algorithms, and code implementations must be traceable to reliable sources.

### Clarity
Content must be readable and understandable for a professional audience interested in AI, robotics, and software engineering. Writing clarity must meet Flesch-Kincaid grade 10–12 or equivalent technical readability standards.

### Reproducibility
All code snippets, workflows, and implementations must be traceable and replicable. Code blocks must include comments explaining functionality to ensure they are ready-to-run.

### Integration
The RAG chatbot must be seamlessly embedded in the published book and correctly retrieve answers from selected book content. Chatbot behavior must only provide answers based on the selected book text and not hallucinate content.

### Modern Deployment
Book must follow best practices in Docusaurus deployment on GitHub Pages. Deployment format must be a Docusaurus static site hosted on GitHub Pages with integrated chatbot functionality.

## Technical Standards

### Citation & Source Management
Citation format: APA style. Minimum 50% of sources must be peer-reviewed or official documentation. Plagiarism tolerance: 0%. All content must pass plagiarism checks before final submission.

### Content Quality
Writing clarity: Flesch-Kincaid grade 10–12 or equivalent technical readability. Content must be factually accurate and technically sound. Visuals, diagrams, and tables should enhance understanding of AI/robotics concepts.

### Code & Chatbot Standards
Code clarity: All code blocks must include comments explaining functionality. Code snippets must be ready-to-run. RAG chatbot responses must strictly reference the selected book text; hallucination is unacceptable. Chatbot tech stack: OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, Qdrant Cloud Free Tier.

## Development Workflow

### Book Structure Requirements
Organized into Modules (minimum 4), each with 3–5 chapters. Chapters should include theory, examples, and practical code demonstrations. All content must follow the established structure and formatting guidelines.

### Source & Reference Standards
Minimum 20 sources. Sources must be credible, with priority given to peer-reviewed articles, official documentation, and authoritative textbooks. All sources must be properly cited and accessible. Content must be original work or properly attributed to avoid plagiarism.

### Deployment Process
Deployment format: Docusaurus static site hosted on GitHub Pages with integrated chatbot functionality. Follow Docusaurus best practices for static site generation and deployment. Ensure chatbot integration works post-deployment on the live site.

## Quality Validation & Testing

### Fact-Checking Requirements
Verify all claims against sources; maintain a fact-checking checklist. All technical content must be validated against authoritative sources. Claims must be traceable to cited sources with proper attribution.

### RAG Chatbot Validation
Test RAG chatbot responses for accuracy (answers correctly based on selected content), relevance (ignores unrelated content), and robustness (handles edge cases and unexpected input). The chatbot must strictly reference book content without hallucination.

### Code Validation
Validate code snippets by running them in the specified environment. All code examples must be tested and confirmed to work as documented. Include error handling and edge case considerations where appropriate.

## Governance

This constitution serves as the governing document for all development activities related to the AI/Spec-Driven Book on Physical AI & Humanoid Robotics with Embedded RAG Chatbot project. All contributors must adhere to these principles and standards. Any deviations must be documented with justification and approved by project leadership. Changes to this constitution require formal amendment procedures with appropriate review and approval.

**Version**: 1.1.0 | **Ratified**: 2025-12-19 | **Last Amended**: 2025-12-19