/**
 * Parse and update book state markdown files.
 *
 * Book state files track ingestion progress per chapter/section.
 * Format: YAML frontmatter + markdown checklist.
 */

import { readFile, writeFile, readdir } from "node:fs/promises";
import { join, resolve } from "node:path";
import matter from "gray-matter";

export interface BookMeta {
  title: string;
  author: string;
  edition?: string;
  file: string;
  status: "not-started" | "in-progress" | "complete";
  "pdf-available"?: boolean;
}

export interface Section {
  /** Raw line text (without checkbox prefix) */
  label: string;
  /** Whether this section has been ingested */
  checked: boolean;
  /** Original line number in the file */
  lineNumber: number;
  /** Page range parsed from label, if present (e.g. "pp. 35-46") */
  pages?: { start: number; end: number };
  /** Group heading this section belongs to */
  group: string;
}

export interface BookState {
  meta: BookMeta;
  sections: Section[];
  /** Absolute path to the book state file */
  filePath: string;
  /** Absolute path to the PDF (resolved from meta.file) */
  pdfPath: string;
  /** Raw file content for rewriting */
  rawContent: string;
}

/** Parse a page range like "pp. 35-46" or "pages 120-135" from a label */
function parsePages(label: string): { start: number; end: number } | undefined {
  const match = label.match(/(?:pp?\.\s*|pages?\s+)(\d+)\s*[-–]\s*(\d+)/i);
  if (!match) return undefined;
  return { start: parseInt(match[1], 10), end: parseInt(match[2], 10) };
}

/** Parse a single book state file */
export async function parseBookState(filePath: string): Promise<BookState> {
  const rawContent = await readFile(filePath, "utf-8");
  const { data, content } = matter(rawContent);
  const meta = data as BookMeta;

  const lines = content.split("\n");
  const sections: Section[] = [];
  let currentGroup = "";

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];

    // Track group headings (### headings)
    const headingMatch = line.match(/^###\s+(.+)/);
    if (headingMatch) {
      currentGroup = headingMatch[1].trim();
      continue;
    }

    // Parse checklist items: - [x] or - [ ]
    const checkMatch = line.match(/^-\s+\[([ xX])\]\s+(.+)/);
    if (checkMatch) {
      const checked = checkMatch[1].toLowerCase() === "x";
      const label = checkMatch[2].trim();
      sections.push({
        label,
        checked,
        lineNumber: i,
        pages: parsePages(label),
        group: currentGroup,
      });
    }
  }

  // Resolve PDF path relative to the book state file's directory
  const dir = filePath.substring(0, filePath.lastIndexOf("/"));
  const pdfPath = resolve(dir, meta.file);

  return { meta, sections, filePath, pdfPath, rawContent };
}

/** Find the next unchecked section in a book state */
export function findNextSection(book: BookState): Section | undefined {
  return book.sections.find((s) => !s.checked);
}

/** Mark a section as checked and update the file */
export async function markSectionComplete(
  book: BookState,
  section: Section,
  knowledgePath?: string
): Promise<void> {
  const lines = book.rawContent.split("\n");

  // Find the actual line in the raw content that matches
  // We search by matching the checkbox pattern and label
  let targetLine = -1;
  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];
    if (line.match(/^-\s+\[ \]/) && line.includes(section.label.substring(0, 30))) {
      targetLine = i;
      break;
    }
  }

  if (targetLine === -1) {
    throw new Error(`Could not find section "${section.label}" in ${book.filePath}`);
  }

  // Replace [ ] with [x], optionally append knowledge path
  let newLine = lines[targetLine].replace("[ ]", "[x]");
  if (knowledgePath) {
    newLine += ` → ${knowledgePath}`;
  }
  lines[targetLine] = newLine;

  // Update status in frontmatter if needed
  const allChecked = book.sections.every(
    (s) => s.checked || s === section
  );
  let newContent = lines.join("\n");

  if (allChecked) {
    newContent = newContent.replace(/^status:\s*.+$/m, "status: complete");
  } else {
    newContent = newContent.replace(/^status:\s*not-started$/m, "status: in-progress");
  }

  await writeFile(book.filePath, newContent, "utf-8");
}

/** Load all book state files from the sources directory */
export async function loadAllBooks(sourcesDir: string): Promise<BookState[]> {
  const files = await readdir(sourcesDir);
  const mdFiles = files.filter((f) => f.endsWith(".md"));

  const books: BookState[] = [];
  for (const file of mdFiles) {
    const filePath = join(sourcesDir, file);
    const book = await parseBookState(filePath);

    // Skip books without available PDFs
    if (book.meta["pdf-available"] === false) {
      console.log(`⏭  Skipping "${book.meta.title}" — PDF not available`);
      continue;
    }

    books.push(book);
  }

  return books;
}

/** Pick the next book and section to ingest */
export function pickNextIngestion(
  books: BookState[]
): { book: BookState; section: Section } | undefined {
  for (const book of books) {
    if (book.meta.status === "complete") continue;
    const section = findNextSection(book);
    if (section) {
      return { book, section };
    }
  }
  return undefined;
}
