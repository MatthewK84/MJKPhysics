//! Source-level Power of Ten checks on engine crates.
//!
//! - Rule 2: every `loop` and `while` carries a `// bound:` comment naming its upper bound.
//! - Rule 8: no `macro_rules!` and no conditional compilation other than `#[cfg(test)]`.
//! - Rule 9: no `dyn` trait objects and no `fn(…)` pointer types.
//! - Rule 5: assertions and functions are counted for a density report.
//!
//! Test code (`#[cfg(test)]` modules and `#[test]` functions) is skipped. The walk uses syn's
//! visitor, which recurses with the depth of the source's own nesting; xtask is tooling and is
//! exempt from rule 1 for that reason (CONTRIBUTING.md).

use std::fmt;

use proc_macro2::Span;
use syn::spanned::Spanned;
use syn::visit::{self, Visit};

/// Lines above a loop searched for its `// bound:` comment.
const BOUND_COMMENT_REACH: usize = 6;

/// The rule a finding breaks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Rule {
    /// Rule 2: loops need a stated bound.
    BoundedLoops,
    /// Rule 8: no `macro_rules!`.
    Macros,
    /// Rule 8: no conditional compilation outside tests.
    ConditionalCompilation,
    /// Rule 9: no trait objects or function pointers.
    Pointers,
}

impl Rule {
    fn label(self) -> &'static str {
        match self {
            Self::BoundedLoops => "rule 2 (bounded loops)",
            Self::Macros => "rule 8 (no macro_rules!)",
            Self::ConditionalCompilation => "rule 8 (no conditional compilation)",
            Self::Pointers => "rule 9 (no trait objects or function pointers)",
        }
    }
}

/// One violation, located in the source.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Finding {
    /// Path relative to the repository root.
    pub file: String,
    /// One-based line.
    pub line: usize,
    /// The rule broken.
    pub rule: Rule,
    /// What was found and how to fix it.
    pub message: String,
}

impl fmt::Display for Finding {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}:{}: {}: {}",
            self.file,
            self.line,
            self.rule.label(),
            self.message
        )
    }
}

/// Functions and assertion macros counted outside tests.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AssertionCount {
    /// Non-test functions with bodies.
    pub functions: usize,
    /// `assert*!` and `debug_assert*!` invocations.
    pub assertions: usize,
}

/// Findings and counts for one file.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FileReport {
    /// Violations found.
    pub findings: Vec<Finding>,
    /// Counts for the assertion-density report.
    pub counts: AssertionCount,
}

/// Checks one engine source file.
///
/// # Errors
///
/// Returns syn's error if the file does not parse.
pub fn check_source(file: &str, text: &str) -> Result<FileReport, syn::Error> {
    let syntax = syn::parse_file(text)?;
    let lines: Vec<&str> = text.lines().collect();
    let mut checker = Checker {
        file,
        lines: &lines,
        report: FileReport::default(),
    };
    checker.visit_file(&syntax);
    Ok(checker.report)
}

struct Checker<'a> {
    file: &'a str,
    lines: &'a [&'a str],
    report: FileReport,
}

fn is_cfg_test(attr: &syn::Attribute) -> bool {
    attr.path().is_ident("cfg")
        && attr
            .parse_args::<syn::Ident>()
            .is_ok_and(|ident| ident == "test")
}

fn is_test_only(attrs: &[syn::Attribute]) -> bool {
    attrs
        .iter()
        .any(|attr| attr.path().is_ident("test") || is_cfg_test(attr))
}

/// True when the loop's line, or one of the comment lines just above it, states a bound.
fn has_bound_comment(lines: &[&str], line: usize) -> bool {
    let same_line = line.checked_sub(1).and_then(|index| lines.get(index));
    if same_line.is_some_and(|text| text.contains("// bound:")) {
        return true;
    }
    let above = lines.get(..line.saturating_sub(1)).unwrap_or_default();
    above
        .iter()
        .rev()
        .take(BOUND_COMMENT_REACH)
        .take_while(|text| text.trim_start().starts_with("//"))
        .any(|text| text.contains("// bound:"))
}

impl Checker<'_> {
    fn push(&mut self, span: Span, rule: Rule, message: &str) {
        self.report.findings.push(Finding {
            file: self.file.to_owned(),
            line: span.start().line,
            rule,
            message: message.to_owned(),
        });
    }

    fn require_bound(&mut self, span: Span, keyword: &str) {
        if !has_bound_comment(self.lines, span.start().line) {
            let message =
                format!("`{keyword}` without a `// bound:` comment stating its upper bound");
            self.push(span, Rule::BoundedLoops, &message);
        }
    }
}

impl<'ast> Visit<'ast> for Checker<'_> {
    fn visit_item_mod(&mut self, node: &'ast syn::ItemMod) {
        if !is_test_only(&node.attrs) {
            visit::visit_item_mod(self, node);
        }
    }

    fn visit_item_fn(&mut self, node: &'ast syn::ItemFn) {
        if !is_test_only(&node.attrs) {
            self.report.counts.functions += 1;
            visit::visit_item_fn(self, node);
        }
    }

    fn visit_impl_item_fn(&mut self, node: &'ast syn::ImplItemFn) {
        if !is_test_only(&node.attrs) {
            self.report.counts.functions += 1;
            visit::visit_impl_item_fn(self, node);
        }
    }

    fn visit_attribute(&mut self, attr: &'ast syn::Attribute) {
        let conditional = attr.path().is_ident("cfg") || attr.path().is_ident("cfg_attr");
        if conditional && !is_cfg_test(attr) {
            self.push(
                attr.span(),
                Rule::ConditionalCompilation,
                "conditional compilation outside tests",
            );
        }
    }

    fn visit_item_macro(&mut self, node: &'ast syn::ItemMacro) {
        if node.mac.path.is_ident("macro_rules") {
            self.push(
                node.span(),
                Rule::Macros,
                "`macro_rules!` in an engine crate",
            );
        }
        visit::visit_item_macro(self, node);
    }

    fn visit_macro(&mut self, node: &'ast syn::Macro) {
        let name = node
            .path
            .segments
            .last()
            .map(|segment| segment.ident.to_string());
        if name.is_some_and(|name| name.starts_with("assert") || name.starts_with("debug_assert")) {
            self.report.counts.assertions += 1;
        }
        visit::visit_macro(self, node);
    }

    fn visit_expr_loop(&mut self, node: &'ast syn::ExprLoop) {
        self.require_bound(node.loop_token.span, "loop");
        visit::visit_expr_loop(self, node);
    }

    fn visit_expr_while(&mut self, node: &'ast syn::ExprWhile) {
        self.require_bound(node.while_token.span, "while");
        visit::visit_expr_while(self, node);
    }

    fn visit_type_trait_object(&mut self, node: &'ast syn::TypeTraitObject) {
        self.push(
            node.span(),
            Rule::Pointers,
            "`dyn` trait object; use generics",
        );
        visit::visit_type_trait_object(self, node);
    }

    fn visit_type_bare_fn(&mut self, node: &'ast syn::TypeBareFn) {
        self.push(
            node.span(),
            Rule::Pointers,
            "function pointer type; use generics",
        );
        visit::visit_type_bare_fn(self, node);
    }
}

#[cfg(test)]
mod tests {
    use super::{Rule, check_source};

    fn rules(text: &str) -> Vec<(usize, Rule)> {
        let report = check_source("sample.rs", text).expect("parses");
        report
            .findings
            .into_iter()
            .map(|f| (f.line, f.rule))
            .collect()
    }

    #[test]
    fn unbounded_loops_are_reported_and_bounded_ones_pass() {
        let text = "fn f(mut n: u32) {\n    while n > 0 { n -= 1; }\n    // bound: at most 3 passes\n    loop { break; }\n    for _ in 0..3 {}\n}\n";
        assert_eq!(rules(text), vec![(2, Rule::BoundedLoops)]);
    }

    #[test]
    fn bound_comment_may_sit_above_other_comments() {
        let text = "fn f() {\n    // bound: one pass\n    // (the body breaks immediately)\n    loop { break; }\n}\n";
        assert_eq!(rules(text), vec![]);
    }

    #[test]
    fn macros_cfg_and_pointers_are_reported() {
        let text = "macro_rules! m { () => {} }\n#[cfg(feature = \"x\")]\nfn f(g: fn(f64) -> f64, h: &dyn Fn()) {}\n";
        assert_eq!(
            rules(text),
            vec![
                (1, Rule::Macros),
                (2, Rule::ConditionalCompilation),
                (3, Rule::Pointers),
                (3, Rule::Pointers)
            ]
        );
    }

    #[test]
    fn test_code_is_skipped() {
        let text = "#[cfg(test)]\nmod tests {\n    macro_rules! m { () => {} }\n    fn f(g: &dyn Fn()) { loop { break; } }\n}\n#[test]\nfn t() { while true {} }\n";
        assert_eq!(rules(text), vec![]);
    }

    #[test]
    fn assertions_and_functions_are_counted() {
        let text = "fn f(x: f64) { debug_assert!(x > 0.0); assert_eq!(1, 1); }\nfn g() {}\n#[cfg(test)]\nmod tests { fn h() { assert!(true); } }\n";
        let counts = check_source("sample.rs", text).expect("parses").counts;
        assert_eq!((counts.functions, counts.assertions), (2, 2));
    }
}
