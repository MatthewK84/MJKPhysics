// @ts-check
// Viewer lint policy. The rationale for each rule is in CONTRIBUTING.md.
import eslint from '@eslint/js';
import { defineConfig } from 'eslint/config';
import tseslint from 'typescript-eslint';

const ASSIGN_TO_GLOBAL =
  'AssignmentExpression > MemberExpression.left[object.name=/^(window|globalThis|global|self)$/]';

export default defineConfig(
  { ignores: ['dist/', 'node_modules/'] },
  eslint.configs.recommended,
  tseslint.configs.strictTypeChecked,
  tseslint.configs.stylisticTypeChecked,
  {
    languageOptions: {
      parserOptions: { projectService: true, tsconfigRootDir: import.meta.dirname },
    },
    rules: {
      // Explicit data shapes and signatures.
      '@typescript-eslint/explicit-function-return-type': 'error',
      '@typescript-eslint/explicit-module-boundary-types': 'error',
      // Simple, shallow control flow and small functions.
      'max-depth': ['error', 3],
      complexity: ['error', 10],
      'max-lines-per-function': ['error', { max: 60, skipBlankLines: true, skipComments: true }],
      // Block scoping and immutability by default.
      'no-var': 'error',
      'prefer-const': 'error',
      'no-param-reassign': ['error', { props: true }],
      // Legacy and unsafe features; no global state; no monkey-patching built-ins.
      'no-eval': 'error',
      'no-new-func': 'error',
      'no-extend-native': 'error',
      'no-restricted-properties': [
        'error',
        { object: 'document', property: 'write', message: 'document.write is banned.' },
      ],
      'no-restricted-syntax': [
        'error',
        { selector: ASSIGN_TO_GLOBAL, message: 'Do not attach state to global objects.' },
      ],
    },
  },
  {
    files: ['**/*.js'],
    extends: [tseslint.configs.disableTypeChecked],
  },
);
