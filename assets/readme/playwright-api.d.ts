// The part of Playwright's API that render-hero.mjs uses. Playwright is not a dependency of this
// repository: the script finds an installed copy at run time and checks its shape before use.

export interface Request {
  url(): string;
}

export interface Route {
  request(): Request;
  fulfill(options: { path: string }): Promise<void>;
  abort(): Promise<void>;
}

export interface Locator {
  screenshot(options: { path: string }): Promise<unknown>;
}

export interface Page {
  on(event: 'pageerror', listener: (error: Error) => void): void;
  route(url: string, handler: (route: Route) => Promise<void>): Promise<void>;
  goto(url: string): Promise<unknown>;
  waitForSelector(selector: string, options: { timeout: number }): Promise<unknown>;
  locator(selector: string): Locator;
}

export interface Browser {
  newPage(options: {
    viewport: { width: number; height: number };
    deviceScaleFactor: number;
  }): Promise<Page>;
  close(): Promise<void>;
}

export interface Playwright {
  chromium: { launch(): Promise<Browser> };
}
