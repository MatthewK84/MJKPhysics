//! Rule 1, no recursion: call-graph cycles in compiled LLVM IR.
//!
//! A source-level call graph needs type inference to resolve method calls; the compiler has
//! already done that. This module reads `rustc --emit=llvm-ir` output from an unoptimized
//! build with one codegen unit, where every direct call is explicit, including calls made
//! through generics and closures after monomorphization. A function is recursive when it can
//! reach itself. Only cycles that contain at least one of our functions are reported, so
//! recursion inside the standard library (for example in `sort`) is ignored unless it calls
//! back into our code.

use std::collections::{BTreeSet, HashMap, HashSet, VecDeque};

/// Functions defined in one LLVM module and the direct calls between them.
#[derive(Debug, Default)]
pub struct CallGraph {
    names: Vec<String>,
    edges: Vec<BTreeSet<usize>>,
}

/// Reads the symbol that follows `@` at the start of `text`, and how many characters it
/// occupies there (two more than its length when quoted).
fn symbol_at(text: &str) -> Option<(&str, usize)> {
    if let Some(quoted) = text.strip_prefix('"') {
        return quoted
            .split_once('"')
            .map(|(name, _)| (name, name.len() + 2));
    }
    let end = text
        .find(|c: char| !(c.is_ascii_alphanumeric() || "._$-".contains(c)))
        .unwrap_or(text.len());
    text.get(..end)
        .filter(|name| !name.is_empty())
        .map(|name| (name, end))
}

/// Symbols called directly on one instruction line: every `@symbol(`. Functions passed as
/// values (`ptr @symbol`) are not calls.
fn called_symbols(line: &str) -> Vec<&str> {
    line.split('@')
        .skip(1)
        .filter_map(|rest| {
            let (name, width) = symbol_at(rest)?;
            rest.get(width..)
                .filter(|after| after.starts_with('('))
                .map(|_| name)
        })
        .collect()
}

/// The symbol a `define` line introduces.
fn defined_name(line: &str) -> Option<&str> {
    let rest = line.strip_prefix("define ")?;
    let (_, tail) = rest.split_once('@')?;
    symbol_at(tail).map(|(name, _)| name)
}

impl CallGraph {
    /// Builds the graph from the text of a `.ll` file.
    #[must_use]
    pub fn from_ir(text: &str) -> Self {
        let mut graph = Self::default();
        let mut index: HashMap<String, usize> = HashMap::new();
        let mut current: Option<usize> = None;
        for line in text.lines() {
            if let Some(name) = defined_name(line) {
                current = Some(graph.node(&mut index, name));
                continue;
            }
            if line.starts_with('}') {
                current = None;
                continue;
            }
            let is_call = line.contains(" call ") || line.contains(" invoke ");
            let Some(caller) = current.filter(|_| is_call) else {
                continue;
            };
            for callee in called_symbols(line) {
                graph.add_call(&mut index, caller, callee);
            }
        }
        graph
    }

    fn node(&mut self, index: &mut HashMap<String, usize>, name: &str) -> usize {
        if let Some(&existing) = index.get(name) {
            return existing;
        }
        let id = self.names.len();
        self.names.push(name.to_owned());
        self.edges.push(BTreeSet::new());
        index.insert(name.to_owned(), id);
        id
    }

    fn add_call(&mut self, index: &mut HashMap<String, usize>, from: usize, to_symbol: &str) {
        let target = self.node(index, to_symbol);
        if let Some(out) = self.edges.get_mut(from) {
            out.insert(target);
        }
    }

    /// Every distinct cycle through at least one function for which `is_ours` holds, as
    /// demangled names, with each cycle's first function repeated at the end.
    #[must_use]
    pub fn recursion(&self, is_ours: impl Fn(&str) -> bool) -> Vec<Vec<String>> {
        let mut reported: BTreeSet<BTreeSet<usize>> = BTreeSet::new();
        let mut cycles = Vec::new();
        for (id, name) in self.names.iter().enumerate() {
            if !is_ours(&demangled(name)) {
                continue;
            }
            let Some(cycle) = cycle_through(&self.edges, id) else {
                continue;
            };
            if reported.insert(cycle.iter().copied().collect()) {
                cycles.push(
                    cycle
                        .iter()
                        .chain(cycle.first())
                        .map(|&n| self.display(n))
                        .collect(),
                );
            }
        }
        cycles
    }

    fn display(&self, id: usize) -> String {
        self.names
            .get(id)
            .map_or_else(String::new, |name| demangled(name))
    }
}

/// A shortest call path from `start` back to itself, if one exists (breadth-first search).
fn cycle_through(edges: &[BTreeSet<usize>], start: usize) -> Option<Vec<usize>> {
    let mut parent: HashMap<usize, usize> = HashMap::new();
    let mut seen: HashSet<usize> = HashSet::new();
    let mut queue: VecDeque<usize> = VecDeque::from([start]);
    // bound: each node enters the queue at most once, so this runs at most once per node.
    while let Some(node) = queue.pop_front() {
        for &next in edges.get(node).into_iter().flatten() {
            if next == start {
                return Some(path_back(&parent, node, start));
            }
            if seen.insert(next) {
                parent.insert(next, node);
                queue.push_back(next);
            }
        }
    }
    None
}

/// Walks parents from `last` back to `start`, returning the cycle in call order.
fn path_back(parent: &HashMap<usize, usize>, last: usize, start: usize) -> Vec<usize> {
    let mut path = vec![last];
    // bound: each step moves one BFS level closer to `start`; there are at most as many
    // levels as nodes.
    while let Some(&previous) = path.last().and_then(|node| parent.get(node)) {
        path.push(previous);
    }
    path.push(start);
    path.reverse();
    path.dedup();
    path
}

/// The demangled symbol without its hash suffix.
#[must_use]
pub fn demangled(symbol: &str) -> String {
    format!("{:#}", rustc_demangle::demangle(symbol))
}

#[cfg(test)]
mod tests {
    use super::{CallGraph, called_symbols};

    const IR: &str = r#"
define void @ours_a() {
  call void @ours_b()
  ret void
}
define void @ours_b() {
  %1 = invoke i32 @ours_a() to label %ok unwind label %bad
  call void @"std_sort"(ptr @ours_c)
}
define void @"std_sort"() {
  call void @"std_sort"()
}
define void @ours_c() {
  call void @ours_c()
}
define void @ours_d() {
  call void @"std_sort"()
}
"#;

    #[test]
    fn calls_are_parsed_but_function_references_are_not() {
        assert_eq!(
            called_symbols("  call void @\"std_sort\"(ptr @ours_c)"),
            vec!["std_sort"]
        );
    }

    #[test]
    fn direct_and_mutual_recursion_in_our_code_is_found() {
        let graph = CallGraph::from_ir(IR);
        let cycles = graph.recursion(|name| name.starts_with("ours_"));
        assert_eq!(
            cycles,
            vec![
                vec![
                    "ours_a".to_owned(),
                    "ours_b".to_owned(),
                    "ours_a".to_owned()
                ],
                vec!["ours_c".to_owned(), "ours_c".to_owned()],
            ]
        );
    }

    #[test]
    fn recursion_only_inside_library_code_is_ignored() {
        let graph = CallGraph::from_ir(IR);
        let cycles = graph.recursion(|name| name == "ours_d");
        assert!(cycles.is_empty());
    }
}
