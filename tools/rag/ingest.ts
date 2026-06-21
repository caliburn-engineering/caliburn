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
import { writeFileSync, readFileSync, existsSync, mkdirSync } from "node:fs";
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

  // Pick a model — prefer local Ollama (free, runs unattended at 3 AM)
  const preferredModels = [
    ["ollama", "qwen3:8b"],
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
  let textBuffer = "";

  // Listen for tool executions to capture the write path, and buffer text output
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
      const delta = event.assistantMessageEvent.delta;
      textBuffer += delta;
      process.stdout.write(delta);
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

  // Fallback: if the model output a knowledge file as text instead of calling the tool,
  // extract the markdown and write it ourselves
  if (!resultPath && textBuffer.includes("---\ntitle:")) {
    console.log("\n📝 Model output knowledge file as text — writing via fallback...");
    resultPath = await writeFallbackKnowledge(textBuffer, book, section, CALIBURN_ROOT);
  }

  return resultPath;
}

/**
 * Fallback: extract a knowledge file from the model's text output and write it.
 * Used when the model outputs markdown instead of calling the write_knowledge tool.
 */
async function writeFallbackKnowledge(
  text: string,
  book: { meta: { title: string; author: string } },
  section: { label: string; group: string; pages?: { start: number; end: number } },
  caliburnRoot: string
): Promise<string | undefined> {
  // Extract markdown between ```markdown ... ``` or starting from ---\ntitle:
  let content: string | undefined;

  // Try fenced code block first
  const fenced = text.match(/```markdown\s*\n(---\n[\s\S]*?\n---[\s\S]*?)```/);
  if (fenced) {
    content = fenced[1].trim();
  } else {
    // Try raw frontmatter block
    const raw = text.match(/(---\ntitle:[\s\S]*)/);
    if (raw) {
      content = raw[1].trim();
      // Remove trailing ``` if present
      content = content.replace(/\n```\s*$/, "").trim();
    }
  }

  if (!content) return undefined;

  // Extract title from frontmatter for filename
  const titleMatch = content.match(/title:\s*(.+)/);
  if (!titleMatch) return undefined;
  const title = titleMatch[1].trim().replace(/^["']|["']$/g, "");

  // Determine placement from the section group
  const group = section.group.toLowerCase();
  let subdir: string;
  if (group.includes("interpolat") || group.includes("integrat") || group.includes("numerical")) {
    subdir = "math/numerical-methods";
  } else if (group.includes("differential") || group.includes("ode")) {
    subdir = "math/numerical-methods";
  } else if (group.includes("linear algebra") || group.includes("eigenvalue")) {
    subdir = "math/linear-algebra";
  } else if (group.includes("control") || group.includes("controller")) {
    subdir = "control-theory";
  } else if (group.includes("simulat") || group.includes("rigid")) {
    subdir = "simulation";
  } else {
    subdir = "math";
  }

  // Generate filename from title
  const filename = title
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "-")
    .replace(/^-|-$/g, "")
    + ".md";

  const relPath = `knowledge/${subdir}/${filename}`;
  const absPath = resolve(caliburnRoot, relPath);

  // Fix requires/related paths to point to actual files
  // Replace invented paths with empty arrays to avoid broken links
  content = content.replace(
    /requires:\n((?:\s+-\s+.*\n)*)/,
    (match, entries) => {
      const lines = entries.trim().split("\n");
      const valid = lines.filter((line: string) => {
        const pathMatch = line.match(/^\s+-\s+"?([^"]+)"?\s*$/);
        if (!pathMatch) return false;
        const refPath = resolve(dirname(absPath), pathMatch[1]);
        return existsSync(refPath);
      });
      return valid.length > 0
        ? `requires:\n${valid.join("\n")}\n`
        : "requires: []\n";
    }
  );
  content = content.replace(
    /related:\n((?:\s+-\s+.*\n)*)/,
    (match, entries) => {
      const lines = entries.trim().split("\n");
      const valid = lines.filter((line: string) => {
        const pathMatch = line.match(/^\s+-\s+"?([^"]+)"?\s*$/);
        if (!pathMatch) return false;
        const refPath = resolve(dirname(absPath), pathMatch[1]);
        return existsSync(refPath);
      });
      return valid.length > 0
        ? `related:\n${valid.join("\n")}\n`
        : "related: []\n";
    }
  );

  // Ensure directory exists
  const dir = dirname(absPath);
  if (!existsSync(dir)) {
    mkdirSync(dir, { recursive: true });
  }

  // Write the file
  writeFileSync(absPath, content, "utf-8");
  console.log(`   Wrote: ${relPath}`);

  // Update or create parent index.md
  const indexPath = resolve(dir, "index.md");
  if (existsSync(indexPath)) {
    const indexContent = readFileSync(indexPath, "utf-8");
    const baseName = filename.replace(".md", "");
    if (!indexContent.includes(baseName)) {
      const entry = `| [${title}](${filename}) | |\n`;
      if (indexContent.includes("| ---")) {
        const lines = indexContent.split("\n");
        let lastTableLine = -1;
        for (let i = lines.length - 1; i >= 0; i--) {
          if (lines[i].startsWith("|") && !lines[i].includes("---")) {
            lastTableLine = i;
            break;
          }
        }
        if (lastTableLine >= 0) {
          lines.splice(lastTableLine + 1, 0, entry.trimEnd());
          writeFileSync(indexPath, lines.join("\n"), "utf-8");
          console.log(`   Updated index: ${subdir}/index.md`);
        }
      }
    }
  } else {
    // Create a new index for new subcategories
    const indexContent = `# ${subdir.split("/").pop()!.replace(/-/g, " ").replace(/\b\w/g, c => c.toUpperCase())}

## Topics

| Topic | Description |
|---|---|
| [${title}](${filename}) | |
`;
    writeFileSync(indexPath, indexContent, "utf-8");
    console.log(`   Created index: ${subdir}/index.md`);

    // Also update the parent math/index.md if we created a new subcategory
    const parentIndexPath = resolve(dir, "../index.md");
    if (existsSync(parentIndexPath)) {
      const parentContent = readFileSync(parentIndexPath, "utf-8");
      const subcatName = subdir.split("/").pop()!;
      if (!parentContent.includes(subcatName)) {
        const subcatTitle = subcatName.replace(/-/g, " ").replace(/\b\w/g, c => c.toUpperCase());
        const entry = `| ${subcatTitle} | [${subcatName}/](${subcatName}/index.md) | |\n`;
        const lines = parentContent.split("\n");
        let lastTableLine = -1;
        for (let i = lines.length - 1; i >= 0; i--) {
          if (lines[i].startsWith("|") && !lines[i].includes("---")) {
            lastTableLine = i;
            break;
          }
        }
        if (lastTableLine >= 0) {
          lines.splice(lastTableLine + 1, 0, entry.trimEnd());
          writeFileSync(parentIndexPath, lines.join("\n"), "utf-8");
          console.log(`   Updated parent index: ${subdir.split("/").slice(0, -1).join("/")}/index.md`);
        }
      }
    }
  }

  return relPath;
}

main().catch((err) => {
  console.error("Fatal error:", err);
  process.exit(1);
});
