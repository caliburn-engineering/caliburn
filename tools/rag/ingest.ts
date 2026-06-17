/**
 * Caliburn RAG Pipeline — Orchestrator
 *
 * Nightly knowledge ingestion from textbook PDFs.
 *
 * Flow:
 * 1. Parse book state files from tools/rag/sources/
 * 2. Find next unchecked section (one section per run)
 * 3. Extract PDF pages for that section (max 20 pages)
 * 4. Spawn Pi agent session to write/merge knowledge files
 * 5. Update book state file with completion status
 *
 * Usage:
 *   npm run ingest          # Run one section
 *   npm run ingest:dry      # Dry run — show what would be ingested
 */

import { resolve, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { loadAllBooks, pickNextIngestion, markSectionComplete } from "./lib/book-state.js";
import { extractSection } from "./lib/pdf-extract.js";
import {
  createAgentSession,
  DefaultResourceLoader,
  getAgentDir,
  SessionManager,
  AuthStorage,
  ModelRegistry,
} from "@earendil-works/pi-coding-agent";

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

const SOURCES_DIR = resolve(__dirname, "sources");
const CALIBURN_ROOT = resolve(__dirname, "../..");

const isDryRun = process.argv.includes("--dry-run");

async function main() {
  console.log("🔍 Scanning book state files...\n");

  // 1. Load all book states
  const books = await loadAllBooks(SOURCES_DIR);
  console.log(`   Found ${books.length} books with available PDFs\n`);

  // 2. Pick next section to ingest
  const next = pickNextIngestion(books);
  if (!next) {
    console.log("✅ All available sections have been ingested!");
    return;
  }

  const { book, section } = next;
  console.log(`📖 Next: "${book.meta.title}" by ${book.meta.author}`);
  console.log(`   Section: ${section.label}`);
  console.log(`   Group: ${section.group}`);
  if (section.pages) {
    console.log(`   Pages: ${section.pages.start}–${section.pages.end}`);
  }
  console.log();

  if (isDryRun) {
    console.log("🏜  Dry run — stopping here.");
    return;
  }

  // 3. Extract PDF pages
  console.log("📄 Extracting PDF pages...");
  const extraction = await extractSection(book.pdfPath, section.pages);
  console.log(
    `   Extracted ${extraction.pageCount} pages (${extraction.text.length} chars)\n`
  );

  if (extraction.text.trim().length < 100) {
    console.log("⚠️  Extracted text too short — may be a scanned PDF. Skipping.");
    return;
  }

  // 4. Spawn Pi agent session to write knowledge
  console.log("🤖 Spawning Pi agent session for knowledge writing...\n");

  const knowledgePath = await writeKnowledge(extraction.text, book, section);

  // 5. Update book state
  if (knowledgePath) {
    console.log(`\n✏️  Marking section complete...`);
    await markSectionComplete(book, section, knowledgePath);
    console.log(`✅ Done: ${section.label} → ${knowledgePath}`);
  } else {
    console.log("\n⚠️  Knowledge writer did not produce output. Section NOT marked complete.");
  }
}

/**
 * Spawn a Pi agent session that writes a knowledge file from extracted text.
 * Returns the path to the created/updated knowledge file, or undefined on failure.
 */
async function writeKnowledge(
  extractedText: string,
  book: { meta: { title: string; author: string } },
  section: { label: string; group: string; pages?: { start: number; end: number } }
): Promise<string | undefined> {
  const authStorage = AuthStorage.create();

  // If ANTHROPIC_API_KEY is in env (e.g., from Claude Code session), use it
  if (process.env.ANTHROPIC_API_KEY) {
    authStorage.setRuntimeApiKey("anthropic", process.env.ANTHROPIC_API_KEY);
  }

  const modelRegistry = ModelRegistry.create(authStorage);

  // Pick a model — prefer Sonnet for cost efficiency on nightly runs
  const preferredModels = [
    ["anthropic", "claude-sonnet-4-5"],
    ["anthropic", "claude-sonnet-4-6"],
    ["google", "gemini-2.5-pro"],
  ] as const;

  let model = undefined;
  for (const [provider, id] of preferredModels) {
    model = modelRegistry.find(provider, id);
    if (model) break;
  }

  // Fall back to any available model
  if (!model) {
    const available = modelRegistry.getAvailable();
    model = available[0];
  }

  if (!model) {
    console.error("❌ No model available. Configure a provider with `pi` first.");
    return undefined;
  }

  // Load the rag-writer extension for the write_knowledge tool
  const ragWriterPath = resolve(CALIBURN_ROOT, ".pi/extensions/rag-writer/index.ts");
  const loader = new DefaultResourceLoader({
    cwd: CALIBURN_ROOT,
    agentDir: getAgentDir(),
    additionalExtensionPaths: [ragWriterPath],
  });
  await loader.reload();

  const { session } = await createAgentSession({
    cwd: CALIBURN_ROOT,
    model,
    thinkingLevel: "low",
    tools: ["read", "write", "edit", "bash", "write_knowledge"],
    resourceLoader: loader,
    sessionManager: SessionManager.inMemory(),
  });

  let resultPath: string | undefined;

  // Listen for tool executions to capture the write path
  session.subscribe((event) => {
    if (event.type === "tool_execution_end" && !event.isError) {
      // Try to capture knowledge file path from write operations
      const result = event.result;
      if (typeof result === "string" && result.includes("knowledge/")) {
        const match = result.match(/(knowledge\/[^\s"']+\.md)/);
        if (match) resultPath = match[1];
      }
    }
    if (event.type === "message_update" && event.assistantMessageEvent.type === "text_delta") {
      process.stdout.write(event.assistantMessageEvent.delta);
    }
  });

  const sourceRef = section.pages
    ? `{ book: "${book.meta.title}", author: "${book.meta.author}", chapter: "${section.label}", pages: "${section.pages.start}-${section.pages.end}" }`
    : `{ book: "${book.meta.title}", author: "${book.meta.author}", chapter: "${section.label}" }`;

  const prompt = `You are the Caliburn knowledge writer. Your job is to create or update a knowledge file from extracted textbook content.

## Source
Book: ${book.meta.title} by ${book.meta.author}
Section: ${section.label}
Group: ${section.group}

## Extracted Text
<extracted-text>
${extractedText}
</extracted-text>

## Instructions

1. Read \`knowledge/index.md\` to understand the existing knowledge tree structure.
2. Determine the correct placement for this content:
   - Navigate the category indexes to find the right folder
   - If a relevant knowledge file already exists, merge into it
   - If not, create a new file
3. Write the knowledge file using this format:

\`\`\`markdown
---
title: <descriptive title>
sources:
  - ${sourceRef}
requires:
  - <relative paths to prerequisite knowledge files>
related:
  - <relative paths to related knowledge files>
---

# <Title>

<Theory content — what it is and why it matters>

## Key Equations

<LaTeX or structured math notation>

## Source Implementation

### ${book.meta.author} — ${section.label}
<Pseudocode or MATLAB from the textbook>

## Implementation Notes

<C++ patterns, Eigen usage, numerical stability considerations>
\`\`\`

4. After creating/updating the knowledge file, update the parent \`index.md\` to include it.
5. Report the path of the created/updated knowledge file.

Write only factual content from the extracted text. Do not invent information.
Focus on content relevant to control engineering, simulation, and linear algebra.`;

  try {
    await session.prompt(prompt);
    // Give the agent a moment to finish tool calls
    await session.agent.waitForIdle();
  } catch (error) {
    console.error("❌ Agent session failed:", error);
  } finally {
    session.dispose();
  }

  return resultPath;
}

main().catch((err) => {
  console.error("Fatal error:", err);
  process.exit(1);
});
