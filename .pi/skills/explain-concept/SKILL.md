---
name: explain-concept
description: Navigate the knowledge tree and explain an engineering concept with prerequisites, equations, and implementation pointers
---

# Explain Concept

Teach a concept from the Caliburn knowledge base. Structure the explanation so the user builds understanding layer by layer: prerequisites first, then the core idea, then practical details.

## Input

The user provides a concept name (e.g. "Kalman filter", "RK4", "eigenvalues", "PID", "state space").

## Procedure

### 1. Navigate the knowledge tree

Read `knowledge/index.md` to get the category table. Identify which category the concept likely belongs to, then read that category's `index.md`. Follow subcategory indexes until you find the target file.

Match flexibly -- the user may say "Kalman filter" but the file is `kalman-filter.md`, or "RK4" for `rk4.md`. Normalise case, expand abbreviations, try synonyms.

### 2. Concept not found

If the concept does not exist in the knowledge base, do NOT fabricate an explanation from general knowledge. Instead:

- List the categories and their available topics (read each category index)
- Suggest the closest match if one exists
- Offer to create a new knowledge file for the concept if the user wants

Stop here.

### 3. Read the knowledge file

Read the matched file in full. Extract from the YAML frontmatter:
- `title` -- display name
- `requires` -- prerequisite file paths (relative)
- `related` -- sibling topic paths
- `sources` -- citation list
- `reference` -- C++ implementation path (if present)

### 4. Load prerequisites

For each path in `requires`, resolve it relative to the knowledge file's location and read that file. Extract the title and a brief summary (first paragraph or key takeaway) from each.

### 5. Deliver the explanation

Structure the response in this exact order:

**Prerequisites.** For each required topic, give a 2-3 sentence summary of what the user needs to know. Frame it as "Before diving in, make sure you're comfortable with..." If a prerequisite itself has prerequisites, mention them but don't recurse -- keep it one level deep.

**Core concept.** Explain what it is and why it matters. Use the knowledge file content as the authoritative source. Prioritise intuition over formalism -- lead with what the thing does, then how.

**Key equations.** Present the essential equations from the knowledge file. Add brief plain-language annotations for each variable or term. Do not add equations that aren't in the knowledge file.

**Practical notes.** Surface implementation notes, tuning guidance, numerical stability warnings, or common pitfalls from the knowledge file. This is the "things that will bite you" section.

**Reference implementation.** If the `reference` field exists in the frontmatter, mention the file path and offer to walk through the code. Do not read the reference file unless the user asks.

**Where to go next.** List the `related` topics with one sentence each explaining the connection. Frame as "From here, you might want to explore..."

**Sources.** Cite every entry from the `sources` frontmatter field. For web sources, include the URL.

### 6. Constraints

- Teach, don't dump. Use plain language before formal notation.
- Stay grounded in the knowledge file content. Do not inject external knowledge that contradicts or extends beyond what the file contains.
- Keep prerequisite summaries brief -- the user can invoke this skill again for any prerequisite they want to go deeper on.
- If the user asks follow-up questions about a related or prerequisite topic, re-invoke this procedure for that topic rather than answering from memory.
