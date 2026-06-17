---
name: rag-writer
description: Knowledge file writer for the RAG ingestion pipeline — validates format, determines placement, updates indexes
---

# RAG Writer Extension

This extension provides the `write_knowledge` tool for creating and updating knowledge files in the Caliburn knowledge base.

## When to use

- Called automatically by the nightly RAG orchestrator (`tools/rag/ingest.ts`)
- Can be invoked manually when writing knowledge files from any source

## Knowledge File Format

```markdown
---
title: Descriptive Title
sources:
  - { book: "Author - Title", chapter: "Ch.N", pages: "X-Y" }
requires:
  - relative/path/to/prerequisite.md
related:
  - relative/path/to/sibling-topic.md
reference: relative/path/to/reference/implementation.cpp
---

# Title

Theory content — what it is, why it matters, where it's used.

## Key Equations

LaTeX or structured math.

## Source Implementations

### Author — Chapter/Section
Pseudocode or MATLAB captured from the textbook.

## Implementation Notes

C++ patterns, Eigen usage, numerical stability notes.
```

## Placement Rules

1. Navigate `knowledge/index.md` to find the right category
2. Read the category's `index.md` to find subcategories
3. Place the file in the most specific applicable folder
4. If no suitable folder exists, create one with its own `index.md`
5. Always update the parent `index.md` after creating a new file
