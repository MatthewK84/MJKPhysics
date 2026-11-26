// @ts-check
/**
 * Renders GitHub-flavored math in the mdBook output with KaTeX (ADR 0005).
 *
 * Pages use the syntax GitHub renders natively, so they read correctly in both places:
 * - display equations are fenced code blocks with the language `math`;
 * - inline equations are code spans wrapped in dollar signs: $`x^2`$.
 * Markdown leaves code untouched, so the TeX reaches KaTeX exactly as written.
 *
 * A display equation ending in \text{(ID)}, such as \text{(CONV-3)}, gets the anchor `eq-ID`
 * so code comments and pages can link to it.
 */
(() => {
  const EQUATION_ID = /\\text\{\(([A-Z][A-Z0-9]*-\d+)\)\}\s*$/;

  /** @type {KatexOptions} */
  const DISPLAY = { displayMode: true, throwOnError: false, trust: false, strict: 'warn' };
  /** @type {KatexOptions} */
  const INLINE = { displayMode: false, throwOnError: false, trust: false, strict: 'warn' };

  /**
   * @param {string} tex
   * @param {HTMLElement} target
   * @param {KatexOptions} options
   * @returns {void}
   */
  function renderInto(tex, target, options) {
    if (katex === undefined) {
      target.replaceChildren(tex);
      target.classList.add('mjk-math-error');
      return;
    }
    katex.render(tex, target, options);
  }

  /**
   * @param {HTMLElement} code a `code.language-math` element inside a `pre`
   * @returns {void}
   */
  function renderDisplay(code) {
    const pre = code.parentElement;
    if (pre === null) {
      return;
    }
    const tex = code.textContent;
    const block = document.createElement('div');
    block.className = 'mjk-math-display';
    const id = EQUATION_ID.exec(tex)?.[1];
    if (id !== undefined) {
      block.id = `eq-${id}`;
    }
    renderInto(tex, block, DISPLAY);
    pre.replaceWith(block);
  }

  /**
   * True when the code span is written as $`...`$.
   * @param {HTMLElement} code
   * @returns {boolean}
   */
  function isInlineMath(code) {
    const before = code.previousSibling;
    const after = code.nextSibling;
    const opens =
      before !== null &&
      before.nodeType === Node.TEXT_NODE &&
      (before.textContent ?? '').endsWith('$');
    const closes =
      after !== null &&
      after.nodeType === Node.TEXT_NODE &&
      (after.textContent ?? '').startsWith('$');
    return opens && closes;
  }

  /**
   * @param {HTMLElement} code
   * @returns {void}
   */
  function renderInline(code) {
    const before = code.previousSibling;
    const after = code.nextSibling;
    if (before === null || after === null) {
      return;
    }
    before.textContent = (before.textContent ?? '').slice(0, -1);
    after.textContent = (after.textContent ?? '').slice(1);
    const span = document.createElement('span');
    span.className = 'mjk-math-inline';
    renderInto(code.textContent, span, INLINE);
    code.replaceWith(span);
  }

  /** @returns {void} */
  function renderPage() {
    document.querySelectorAll('pre > code.language-math').forEach((node) => {
      if (node instanceof HTMLElement) {
        renderDisplay(node);
      }
    });
    document.querySelectorAll(':not(pre) > code').forEach((node) => {
      if (node instanceof HTMLElement && isInlineMath(node)) {
        renderInline(node);
      }
    });
    const target = window.location.hash.slice(1);
    const anchor = target === '' ? null : document.getElementById(target);
    if (anchor !== null) {
      anchor.scrollIntoView();
    }
  }

  renderPage();
})();
