//! The branch protection rule must require exactly the checks that run on a pull request.
//!
//! `.github/rulesets/main.json` is the ruleset that protects `main`. Its required status checks
//! name GitHub Actions jobs. If a job is renamed, the rule waits forever for a check that never
//! reports; if a job is added, it runs but nothing requires it. This check compares the two: every
//! job of every workflow that runs on `pull_request`, unless the job has its own `if:` condition,
//! must be required, and every required check must be such a job.
//!
//! Workflows are read with a narrow line parser rather than a YAML library: it accepts the layout
//! these files use (two-space indents, a `name:` on every job) and reports anything else as a
//! problem instead of guessing.

use std::collections::BTreeSet;
use std::fs;
use std::path::Path;

use serde_json::Value;

/// The ruleset file.
const RULESET: &str = ".github/rulesets/main.json";

/// The directory of workflow files.
const WORKFLOWS: &str = ".github/workflows";

/// The GitHub Actions app, which reports every check.
const GITHUB_ACTIONS_APP: u64 = 15_368;

/// A job of a workflow, as the parser reads it.
#[derive(Debug, Default, PartialEq, Eq)]
struct Job {
    id: String,
    name: Option<String>,
    conditional: bool,
}

/// A workflow file's path and what the parser made of it.
type ParsedFile = (String, Result<Workflow, String>);

/// What the check needs from a workflow file.
#[derive(Debug, Default, PartialEq, Eq)]
struct Workflow {
    on_pull_request: bool,
    jobs: Vec<Job>,
}

/// Every difference between the ruleset and the workflows, described for a person.
///
/// # Errors
///
/// Returns a description when a file cannot be read or the ruleset is not valid JSON.
pub fn check(root: &Path) -> Result<Vec<String>, String> {
    let text = fs::read_to_string(root.join(RULESET)).map_err(|e| format!("{RULESET}: {e}"))?;
    let ruleset: Value = serde_json::from_str(&text).map_err(|e| format!("{RULESET}: {e}"))?;
    let mut problems = ruleset_problems(&ruleset);
    let mut jobs = BTreeSet::new();
    for (file, workflow) in read_workflows(&root.join(WORKFLOWS))? {
        match workflow {
            Ok(parsed) => jobs.extend(pull_request_jobs(&file, &parsed, &mut problems)),
            Err(problem) => problems.push(format!("{file}: {problem}")),
        }
    }
    problems.extend(differences(&required_checks(&ruleset), &jobs));
    Ok(problems)
}

/// Each workflow file's name and what the parser made of it, in file-name order.
fn read_workflows(dir: &Path) -> Result<Vec<ParsedFile>, String> {
    let entries = fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut files: Vec<_> = entries
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "yml"))
        .collect();
    files.sort();
    let mut workflows = Vec::with_capacity(files.len());
    for path in files {
        let name = format!(
            "{WORKFLOWS}/{}",
            path.file_name()
                .map_or_else(String::new, |n| n.to_string_lossy().into_owned())
        );
        let text = fs::read_to_string(&path).map_err(|e| format!("{name}: {e}"))?;
        workflows.push((name, parse_workflow(&text)));
    }
    Ok(workflows)
}

/// The names of the jobs that report a check on every pull request.
fn pull_request_jobs(file: &str, workflow: &Workflow, problems: &mut Vec<String>) -> Vec<String> {
    if !workflow.on_pull_request {
        return Vec::new();
    }
    let mut names = Vec::new();
    for job in workflow.jobs.iter().filter(|job| !job.conditional) {
        match &job.name {
            Some(name) if name.contains("${{") => problems.push(format!(
                "{file}: job `{}` has a computed name, which a ruleset cannot list",
                job.id
            )),
            Some(name) => names.push(name.clone()),
            None => problems.push(format!("{file}: job `{}` has no `name:`", job.id)),
        }
    }
    names
}

/// Required checks missing from the workflows, and pull-request jobs the ruleset leaves out.
fn differences(required: &BTreeSet<String>, jobs: &BTreeSet<String>) -> Vec<String> {
    let mut found: Vec<String> = required
        .difference(jobs)
        .map(|name| format!("{RULESET}: requires `{name}`, which no pull-request job reports"))
        .collect();
    found.extend(jobs.difference(required).map(|name| {
        format!("{RULESET}: job `{name}` runs on every pull request but is not required")
    }));
    found
}

/// The contexts of the ruleset's required status checks.
fn required_checks(ruleset: &Value) -> BTreeSet<String> {
    status_checks(ruleset)
        .iter()
        .filter_map(|check| check.get("context").and_then(Value::as_str))
        .map(str::to_owned)
        .collect()
}

/// The ruleset's required status check entries.
fn status_checks(ruleset: &Value) -> Vec<Value> {
    rules(ruleset)
        .iter()
        .filter(|rule| rule.get("type").and_then(Value::as_str) == Some("required_status_checks"))
        .filter_map(|rule| rule.pointer("/parameters/required_status_checks"))
        .filter_map(Value::as_array)
        .flatten()
        .cloned()
        .collect()
}

fn rules(ruleset: &Value) -> Vec<Value> {
    ruleset
        .get("rules")
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default()
}

/// Settings that would weaken the rule without changing its checks.
fn ruleset_problems(ruleset: &Value) -> Vec<String> {
    let mut problems = Vec::new();
    if ruleset.get("enforcement").and_then(Value::as_str) != Some("active") {
        problems.push(format!("{RULESET}: `enforcement` must be \"active\""));
    }
    if ruleset
        .get("bypass_actors")
        .and_then(Value::as_array)
        .is_none_or(|actors| !actors.is_empty())
    {
        problems.push(format!("{RULESET}: `bypass_actors` must be an empty list"));
    }
    let wrong_app = status_checks(ruleset)
        .iter()
        .filter(|check| {
            check.get("integration_id").and_then(Value::as_u64) != Some(GITHUB_ACTIONS_APP)
        })
        .count();
    if wrong_app > 0 {
        problems.push(format!(
            "{RULESET}: {wrong_app} required check(s) not attributed to GitHub Actions ({GITHUB_ACTIONS_APP})"
        ));
    }
    if status_checks(ruleset).is_empty() {
        problems.push(format!("{RULESET}: no required status checks"));
    }
    problems
}

/// The top-level section a line belongs to.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Section {
    On,
    Jobs,
    Other,
}

/// Reads the triggers and the jobs of a workflow file.
fn parse_workflow(text: &str) -> Result<Workflow, String> {
    let mut workflow = Workflow::default();
    let mut section = Section::Other;
    for (number, line) in text.lines().enumerate() {
        let content = line.trim_start();
        if content.is_empty() || content.starts_with('#') {
            continue;
        }
        if line.starts_with('\t') {
            return Err(format!("line {}: indented with a tab", number + 1));
        }
        let indent = line.len() - content.len();
        if indent == 0 {
            section = top_level(content, &mut workflow);
            continue;
        }
        read_line(section, indent, content, &mut workflow);
    }
    Ok(workflow)
}

/// The section a top-level key opens; `on: [push, pull_request]` is read in place.
fn top_level(content: &str, workflow: &mut Workflow) -> Section {
    if let Some(inline) = content.strip_prefix("on:") {
        workflow.on_pull_request |= inline.contains("pull_request");
        return Section::On;
    }
    if content.starts_with("jobs:") {
        return Section::Jobs;
    }
    Section::Other
}

/// Applies one indented line to the workflow.
fn read_line(section: Section, indent: usize, content: &str, workflow: &mut Workflow) {
    match (section, indent) {
        (Section::On, 2) => {
            workflow.on_pull_request |= content.starts_with("pull_request");
        }
        (Section::Jobs, 2) => {
            let id = content.trim_end_matches(':').trim().to_owned();
            workflow.jobs.push(Job {
                id,
                ..Job::default()
            });
        }
        (Section::Jobs, 4) => read_job_key(content, workflow.jobs.last_mut()),
        _ => {}
    }
}

/// Reads a job's `name:` and notes an `if:`.
fn read_job_key(content: &str, job: Option<&mut Job>) {
    let Some(job) = job else {
        return;
    };
    if let Some(name) = content.strip_prefix("name:") {
        let name = name.trim().trim_matches(|c| c == '"' || c == '\'');
        job.name.get_or_insert_with(|| name.to_owned());
    } else if content.starts_with("if:") {
        job.conditional = true;
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::{differences, parse_workflow, pull_request_jobs, ruleset_problems};

    const WORKFLOW: &str = "\
name: CI
on:
  push:
    branches: [main]
  pull_request:

jobs:
  fmt:
    name: Rust format
    steps:
      - name: not a job name
        if: always()
  deploy:
    name: \"Publish\"
    if: github.ref == 'refs/heads/main'
";

    fn set(names: &[&str]) -> BTreeSet<String> {
        names.iter().map(|name| (*name).to_owned()).collect()
    }

    #[test]
    fn jobs_and_triggers_are_read() {
        let workflow = parse_workflow(WORKFLOW).expect("parses");
        assert!(workflow.on_pull_request);
        let names: Vec<_> = workflow
            .jobs
            .iter()
            .map(|job| (job.name.as_deref(), job.conditional))
            .collect();
        assert_eq!(
            names,
            [(Some("Rust format"), false), (Some("Publish"), true)]
        );
        let mut problems = Vec::new();
        assert_eq!(
            pull_request_jobs("ci.yml", &workflow, &mut problems),
            ["Rust format"]
        );
        assert!(problems.is_empty(), "{problems:?}");
    }

    #[test]
    fn scheduled_workflows_and_inline_triggers_are_told_apart() {
        let scheduled = parse_workflow(
            "on:\n  schedule:\n    - cron: \"1 1 * * 1\"\njobs:\n  a:\n    name: A\n",
        )
        .expect("parses");
        assert!(!scheduled.on_pull_request);
        let inline =
            parse_workflow("on: [push, pull_request]\njobs:\n  a:\n    name: A\n").expect("parses");
        assert!(inline.on_pull_request);
    }

    #[test]
    fn layouts_the_ruleset_cannot_follow_are_reported() {
        assert!(parse_workflow("on:\n\tpull_request:\n").is_err());
        let workflow = parse_workflow("on:\n  pull_request:\njobs:\n  a:\n    name: T (${{ matrix.os }})\n  b:\n    runs-on: x\n").expect("parses");
        let mut problems = Vec::new();
        assert!(pull_request_jobs("w.yml", &workflow, &mut problems).is_empty());
        assert_eq!(problems.len(), 2, "{problems:?}");
        assert!(problems.iter().any(|p| p.contains("computed name")));
        assert!(problems.iter().any(|p| p.contains("no `name:`")));
    }

    #[test]
    fn missing_and_unrequired_checks_are_reported() {
        let found = differences(&set(&["Clippy", "Old name"]), &set(&["Clippy", "New job"]));
        assert_eq!(found.len(), 2, "{found:?}");
        assert!(found.iter().any(|f| f.contains("requires `Old name`")));
        assert!(
            found
                .iter()
                .any(|f| f.contains("`New job` runs on every pull request"))
        );
    }

    #[test]
    fn a_weakened_ruleset_is_reported() {
        let strong = serde_json::json!({
            "enforcement": "active",
            "bypass_actors": [],
            "rules": [{"type": "required_status_checks", "parameters": {"required_status_checks": [{"context": "A", "integration_id": 15368}]}}]
        });
        assert!(ruleset_problems(&strong).is_empty());
        let weak = serde_json::json!({
            "enforcement": "evaluate",
            "bypass_actors": [{"actor_type": "RepositoryRole"}],
            "rules": [{"type": "required_status_checks", "parameters": {"required_status_checks": [{"context": "A"}]}}]
        });
        assert_eq!(ruleset_problems(&weak).len(), 3);
        assert_eq!(ruleset_problems(&serde_json::json!({})).len(), 3);
    }
}
