/**
 * PDF page extraction utility.
 *
 * Extracts text from a range of pages in a PDF file.
 * Uses pdf-parse for text extraction.
 */

import { readFile } from "node:fs/promises";
import { existsSync } from "node:fs";
import pdfParse from "pdf-parse";

export interface ExtractionResult {
  text: string;
  pageCount: number;
  pdfPath: string;
  pageRange: { start: number; end: number };
}

/**
 * Extract text from specific pages of a PDF.
 *
 * @param pdfPath - Absolute path to the PDF file
 * @param startPage - First page to extract (1-indexed)
 * @param endPage - Last page to extract (1-indexed, inclusive)
 * @param maxPages - Maximum number of pages to extract (default: 20)
 */
export async function extractPages(
  pdfPath: string,
  startPage: number,
  endPage: number,
  maxPages: number = 20
): Promise<ExtractionResult> {
  if (!existsSync(pdfPath)) {
    throw new Error(`PDF not found: ${pdfPath}`);
  }

  // Clamp page range to maxPages
  const clampedEnd = Math.min(endPage, startPage + maxPages - 1);

  const buffer = await readFile(pdfPath);

  // pdf-parse options to limit page range
  const options = {
    // Custom page renderer that only returns text for pages in range
    pagerender: (pageData: any) => {
      const pageNum = pageData.pageIndex + 1; // pageIndex is 0-based
      if (pageNum >= startPage && pageNum <= clampedEnd) {
        return pageData.getTextContent().then((textContent: any) => {
          return textContent.items
            .map((item: any) => item.str)
            .join(" ");
        });
      }
      return Promise.resolve("");
    },
  };

  const result = await pdfParse(buffer, options);

  // Clean up extracted text
  const text = result.text
    .split("\n")
    .filter((line: string) => line.trim().length > 0)
    .join("\n");

  return {
    text,
    pageCount: clampedEnd - startPage + 1,
    pdfPath,
    pageRange: { start: startPage, end: clampedEnd },
  };
}

/**
 * Extract text for a section that has a page range in its label.
 *
 * Falls back to extracting a default range if no page info is available.
 */
export async function extractSection(
  pdfPath: string,
  pages?: { start: number; end: number },
  fallbackStartPage: number = 1,
  fallbackPageCount: number = 15
): Promise<ExtractionResult> {
  if (pages) {
    return extractPages(pdfPath, pages.start, pages.end);
  }

  // No page range — extract fallback range
  return extractPages(
    pdfPath,
    fallbackStartPage,
    fallbackStartPage + fallbackPageCount - 1
  );
}
