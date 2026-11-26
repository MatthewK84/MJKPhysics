/** The subset of the KaTeX browser API used by math.js (loaded from the CDN in head.hbs). */
interface KatexOptions {
  displayMode: boolean;
  throwOnError: boolean;
  trust: boolean;
  strict: 'warn' | 'ignore' | 'error';
}

interface KatexApi {
  render(tex: string, element: HTMLElement, options: KatexOptions): void;
}

declare const katex: KatexApi | undefined;
