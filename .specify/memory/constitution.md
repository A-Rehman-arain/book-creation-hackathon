<!-- Sync Impact Report:
Version change: N/A -> 1.0.0
Added sections: All principles and sections based on project requirements
Removed sections: None (new file)
Templates requiring updates: ⚠ pending review - plan-template.md has Constitution Check section that may need updates based on new principles
Follow-up TODOs: None
-->
# AI/Spec-Driven Technical Book with Integrated RAG Chatbot Constitution

## Core Principles

### Spec-Driven Development using Spec-Kit Plus
All features and architecture must be defined in formal specs before implementation; Code changes must follow documented specifications; Every major concept includes explanation and practical example

### Accuracy and Strict Grounding in Book Content
RAG chatbot responses must cite source sections and refuse answers not found in content; No external knowledge or hallucinated answers allowed; Responses must be strictly based on indexed book content only

### Clarity for Software Engineers and AI Practitioners
Writing must be technical, concise, and implementation-focused; Documentation built with Docusaurus and deployed to GitHub Pages; Clear explanations with practical examples for each concept

### Reproducibility of Book Build and Deployment
All infrastructure must be free-tier compatible; Fully reproducible setup with documented commands; Deterministic chunking and embedding pipeline

### Modular Separation of Content, Infrastructure, and RAG Logic
Clear separation between book content, infrastructure, and RAG components; Backend uses FastAPI, vector store uses Qdrant Cloud, database uses Neon serverless Postgres

### Free-Tier Compatible Infrastructure
All infrastructure must use free-tier services only; No proprietary or undocumented APIs; All secrets via environment variables

## RAG Chatbot Requirements
Backend: FastAPI; LLM interface: OpenAI Agents / ChatKit SDK; Vector store: Qdrant Cloud (free tier); Database: Neon serverless Postgres; Embeddings generated from book content only; Supports full-book Q&A and user-selected text-only Q&A

## Development Standards
Book written using Claude Code under Spec-Kit Plus governance; Writing must be technical, concise, and implementation-focused; Every major concept includes explanation and practical example

## Governance
All changes must follow Spec-Kit Plus governance; Code reviews must verify compliance with all principles; Book successfully builds and deploys to GitHub Pages; RAG chatbot answers strictly from indexed book content; Selected-text Q&A mode works with strict grounding

**Version**: 1.0.0 | **Ratified**: 2026-01-01 | **Last Amended**: 2026-01-01