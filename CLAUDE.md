# Project memory

## Workflow: pull requests only

`main` is protected by `.github/rulesets/main.json`: nothing is pushed to it directly. Each change
goes on a short-lived branch from `main` and arrives as a pull request with auto-merge turned on
(rebase). It merges once all fourteen required checks pass, and GitHub deletes the branch. Keep one
pull request open at a time, so each is tested against the current `main`. If a job in
`.github/workflows/` is added, renamed or removed, update the ruleset file in the same pull request;
`cargo xtask standards` fails until they agree, and a repository admin re-imports the file.

## Language: American English only

Always write American English, never British English, in everything this project produces:
code identifiers and comments, documentation and the book (`docs/`), the README, CONTRIBUTING,
commit messages, and GitHub issues, pull requests and comments.

CI enforces this: `typos` with the US locale (`typos.toml`) fails the build on British spellings,
including inside identifiers.

Common corrections:

<!-- typos: off -->
| Write | Not |
|---|---|
| center, meter, fiber | centre, metre, fibre |
| color, behavior, honor, labor, favor | colour, behaviour, honour, labour, favour |
| modeled, modeling, labeled, canceled, traveled | modelled, modelling, labelled, cancelled, travelled |
| analyze, organize, recognize, normalize, linearize, initialize | analyse, organise, recognise, normalise, linearise, initialise |
| license (noun and verb), defense, practice (noun and verb) | licence, defence, practise |
| disk (rotor disk, disk loading), gray, aluminum, maneuver, airplane | disc, grey, aluminium, manoeuvre, aeroplane |
| acknowledgment, judgment, toward, afterward | acknowledgement, judgement, towards, afterwards |
<!-- typos: on -->

Exceptions: keep the published spelling of quoted titles in references, proper nouns and names,
and identifiers defined by external code or standards.

`typos` cannot catch every case. *Disc* is also an American word (a compact disc, a disc brake), so
it passes the check: write *disk* for the rotor disk and disk loading, and look for it in review.
