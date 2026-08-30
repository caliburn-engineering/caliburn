import { defineConfig } from 'astro/config';

export default defineConfig({
  output: 'static',
  // Emscripten bundles live in public/ and are copied verbatim — never processed.
  build: { assets: '_astro' },
});
