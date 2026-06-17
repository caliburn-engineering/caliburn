/**
 * Caliburn RAG Writer — Pi Extension
 *
 * Provides a custom `write_knowledge` tool that validates knowledge file
 * format, determines correct placement in the knowledge tree, and ensures
 * parent indexes are updated.
 *
 * Used by the nightly RAG orchestrator (tools/rag/ingest.ts) and available
 * for manual invocation within Pi sessions.
 */

import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { Type } from "typebox";
import { readFileSync, writeFileSync, existsSync, mkdirSync } from "node:fs";
import { resolve, dirname, relative } from "node:path";

const REQUIRED_FRONTMATTER = ["title", "sources"];
const VALID_CATEGORIES = ["control-theory", "math", "simulation"];

export default function ragWriter(pi: ExtensionAPI) {
  pi.registerTool({
    name: "write_knowledge",
    label: "Write Knowledge",
    description: `Write or update a Caliburn knowledge file. Validates frontmatter format, ensures correct placement in the knowledge tree, and updates the parent index.md.

Use this tool instead of the raw write tool when creating knowledge files to ensure consistency.

Required frontmatter fields: title, sources
Optional frontmatter fields: requires, related, reference

The file path must be under knowledge/ and end with .md.`,
    parameters: Type.Object({
      path: Type.String({
        description:
          'Relative path from repo root, e.g. "knowledge/math/linear-algebra/eigenvalues.md"',
      }),
      content: Type.String({
        description: "Full markdown content including YAML frontmatter",
      }),
      update_index: Type.Boolean({
        description: "Whether to add an entry to the parent index.md (default: true)",
        default: true,
      }),
    }),
    execute: async (_toolCallId, params) => {
      const { path, content, update_index } = params;
      const cwd = process.cwd();
      const absPath = resolve(cwd, path);

      // Validate path is under knowledge/
      if (!path.startsWith("knowledge/")) {
        return {
          content: [
            {
              type: "text" as const,
              text: `Error: Path must be under knowledge/. Got: ${path}`,
            },
          ],
          details: {},
        };
      }

      if (!path.endsWith(".md")) {
        return {
          content: [
            { type: "text" as const, text: `Error: File must end with .md. Got: ${path}` },
          ],
          details: {},
        };
      }

      // Validate frontmatter
      const fmMatch = content.match(/^---\n([\s\S]*?)\n---/);
      if (!fmMatch) {
        return {
          content: [
            {
              type: "text" as const,
              text: "Error: Content must start with YAML frontmatter (--- delimiters)",
            },
          ],
          details: {},
        };
      }

      const frontmatter = fmMatch[1];
      const missingFields = REQUIRED_FRONTMATTER.filter(
        (field) => !frontmatter.includes(`${field}:`)
      );
      if (missingFields.length > 0) {
        return {
          content: [
            {
              type: "text" as const,
              text: `Error: Missing required frontmatter fields: ${missingFields.join(", ")}`,
            },
          ],
          details: {},
        };
      }

      // Write the file
      const dir = dirname(absPath);
      if (!existsSync(dir)) {
        mkdirSync(dir, { recursive: true });
      }

      const isUpdate = existsSync(absPath);
      writeFileSync(absPath, content, "utf-8");

      const messages: string[] = [
        `${isUpdate ? "Updated" : "Created"}: ${path}`,
      ];

      // Update parent index.md
      if (update_index !== false) {
        const parentDir = dirname(absPath);
        const indexPath = resolve(parentDir, "index.md");

        if (existsSync(indexPath)) {
          const indexContent = readFileSync(indexPath, "utf-8");
          const fileName = path.split("/").pop()!.replace(".md", "");

          // Check if already listed
          if (!indexContent.includes(fileName)) {
            // Extract title from frontmatter
            const titleMatch = frontmatter.match(/title:\s*(.+)/);
            const title = titleMatch
              ? titleMatch[1].trim().replace(/^["']|["']$/g, "")
              : fileName;

            const entry = `| [${title}](${fileName}.md) | |\n`;

            // Try to append to an existing table, or add at the end
            if (indexContent.includes("| ---")) {
              // Append after the last table row
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
                messages.push(`Updated index: ${relative(cwd, indexPath)}`);
              }
            } else {
              // Just append a bullet point
              const appendLine = `\n- [${title}](${fileName}.md)\n`;
              writeFileSync(
                indexPath,
                indexContent.trimEnd() + "\n" + appendLine,
                "utf-8"
              );
              messages.push(`Updated index: ${relative(cwd, indexPath)}`);
            }
          } else {
            messages.push("Index already contains this entry");
          }
        } else {
          messages.push(`Warning: No index.md found at ${relative(cwd, parentDir)}`);
        }
      }

      return {
        content: [{ type: "text" as const, text: messages.join("\n") }],
        details: {},
      };
    },
  });
}
