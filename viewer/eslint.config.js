// @ts-check
// Lint policy for the viewer and for the repository's other browser and Node scripts
// (docs/theme, assets/readme). Rules are grouped by the Power of Ten principle they enforce; the
// mapping, and what review covers instead, is in CONTRIBUTING.md ("Engineering standards").
//
// Patterns use `**/` so they match whether ESLint runs here or from the repository root with
// `--config viewer/eslint.config.js` (npm run lint:scripts).
import eslint from '@eslint/js';
import prettier from 'eslint-config-prettier';
import { defineConfig } from 'eslint/config';
import tseslint from 'typescript-eslint';

const ASSIGN_TO_GLOBAL =
  'AssignmentExpression > MemberExpression.left[object.name=/^(window|globalThis|global|self)$/]';

export default defineConfig(
  { ignores: ['**/dist/', '**/node_modules/', '**/book/'] },
  // Inline `eslint-disable` comments are ignored and reported, so no file can switch a rule off.
  { linterOptions: { noInlineConfig: true, reportUnusedDisableDirectives: 'error' } },
  eslint.configs.recommended,
  tseslint.configs.strictTypeChecked,
  tseslint.configs.stylisticTypeChecked,
  // Formatting belongs to Prettier; this turns off the stylistic rules that would conflict.
  prettier,
  {
    languageOptions: {
      parserOptions: { projectService: true, tsconfigRootDir: import.meta.dirname },
    },
    rules: {
      // 1. Simple control flow.
      'max-depth': ['error', 3],
      complexity: ['error', 10],
      'no-else-return': 'error',
      // 2. No global or shared mutable state.
      'no-var': 'error',
      'prefer-const': 'error',
      'no-param-reassign': ['error', { props: true }],
      'no-restricted-syntax': [
        'error',
        { selector: ASSIGN_TO_GLOBAL, message: 'Do not attach state to global objects.' },
      ],
      // 3. No unsafe or legacy features.
      'no-eval': 'error',
      'no-new-func': 'error',
      'no-with': 'error',
      'no-implied-eval': 'off',
      '@typescript-eslint/no-implied-eval': 'error',
      'no-restricted-globals': ['error', 'event', 'fdescribe'],
      'prefer-rest-params': 'error',
      'no-restricted-properties': [
        'error',
        { object: 'document', property: 'write', message: 'document.write is banned.' },
      ],
      '@typescript-eslint/no-explicit-any': 'error',
      '@typescript-eslint/no-non-null-assertion': 'error',
      '@typescript-eslint/ban-ts-comment': [
        'error',
        {
          'ts-check': false,
          'ts-expect-error': true,
          'ts-ignore': true,
          'ts-nocheck': true,
        },
      ],
      // 4. Tight scoping.
      'no-shadow': 'off',
      '@typescript-eslint/no-shadow': 'error',
      '@typescript-eslint/no-unused-vars': [
        'error',
        { argsIgnorePattern: '^_', varsIgnorePattern: '^_' },
      ],
      'block-scoped-var': 'error',
      // 7. Small functions.
      'max-lines-per-function': ['error', { max: 50, skipBlankLines: true, skipComments: true }],
      // 8. Explicit data declarations.
      '@typescript-eslint/explicit-function-return-type': 'error',
      '@typescript-eslint/explicit-module-boundary-types': 'error',
      '@typescript-eslint/consistent-type-assertions': [
        'error',
        { assertionStyle: 'as', objectLiteralTypeAssertions: 'never' },
      ],
      // 9. Consistent error handling; no floating promises.
      'no-throw-literal': 'off',
      '@typescript-eslint/only-throw-error': 'error',
      '@typescript-eslint/no-floating-promises': 'error',
      '@typescript-eslint/no-misused-promises': 'error',
      'prefer-promise-reject-errors': 'off',
      '@typescript-eslint/prefer-promise-reject-errors': 'error',
      // 10. No changes to built-ins.
      'no-extend-native': 'error',
      'no-prototype-builtins': 'error',
    },
  },
  {
    // JavaScript files declare parameter and return types in JSDoc, which `tsc --checkJs`
    // enforces; these two rules only recognise TypeScript annotations. tsc also reports
    // undefined names, with knowledge of each file's environment (browser or Node).
    files: ['**/*.js', '**/*.mjs'],
    rules: {
      'no-undef': 'off',
      '@typescript-eslint/explicit-function-return-type': 'off',
      '@typescript-eslint/explicit-module-boundary-types': 'off',
    },
  },
  {
    // This file is not part of a TypeScript project.
    files: ['**/eslint.config.js'],
    extends: [tseslint.configs.disableTypeChecked],
  },
  {
    files: ['**/*.test.ts'],
    rules: { 'max-lines-per-function': 'off' },
  },
);
