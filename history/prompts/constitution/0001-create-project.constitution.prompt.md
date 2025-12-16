---
id: 0001
title: Create Project Constitution
stage: constitution
date: 2025-12-15
surface: agent
model: unspecified
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: [constitution, principles, governance, robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.constitution

Project: Physical AI & Humanoid Robotics Book + Integrated RAG Chatbot
Tools: Docusaurus, Spec-Kit Plus, Claude Code, FastAPI, OpenAI Agents/ChatKit, Neon Postgres, Qdrant

Core Principles:
- Technical accuracy (ROS 2, Gazebo, Unity, Isaac, VLA)
- Clarity for intermediate AI/robotics learners
- Reproducible code and pipelines
- No hallucinated APIs or robotics claims
- Spec-first, consistent structure across all chapters

Standards:
- Sources: official docs or reputable robotics/AI references
- Inline citations only (links or short refs)
- All code runnable on Ubuntu + ROS 2 Humble
- All diagrams must reflect real ROS 2/Isaac/RAG architecture
- Writing clarity: grade 9–12

Book Requirements:
- Docusaurus site deployable to GitHub Pages
- Mandatory modules: ROS 2, Digital Twin, Isaac Sim, VLA, Capstone
- Each chapter must include learning goals, explanations, tested code, diagrams, exercises



Constraints:
- No speculation or unverified robotics claims
- No filler; all sections must be complete and accurate

Success Criteria:
- Book builds and deploys without errors

- All code examples run as documented
- Architecture, APIs, and pipelines verified for correctness

## Response snapshot

The constitution has been successfully updated with the following details:

**New Version**: 1.0.0 (Initial version for the Physical AI & Humanoid Robotics Book project)
**Bump Rationale**: This is the initial creation of the project constitution, establishing core principles for the Physical AI & Humanoid Robotics Book + Integrated RAG Chatbot project.

**Key Changes**:
- Created 6 core principles focusing on technical accuracy, clarity for learners, reproducible code, avoiding hallucinated claims, spec-first consistency, and verified architecture
- Added specific standards and requirements for the robotics book project
- Defined book requirements including Docusaurus deployment and required modules
- Set governance rules for compliance verification

**Files Flagged for Manual Follow-up**:
- .specify/templates/plan-template.md: Constitution Check section may need project-specific updates

**Suggested Commit Message**:
`docs: create constitution for Physical AI & Humanoid Robotics Book (initial version with core principles)`

The constitution has been validated and saved to `.specify/memory/constitution.md`.

## Outcome

- ✅ Impact: Created initial project constitution with 6 core principles and governance rules
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md updated with new constitution
- 🔁 Next prompts: Consider creating feature specifications based on these principles
- 🧠 Reflection: The constitution establishes a solid foundation for the robotics book project with clear guidelines for technical accuracy and reproducibility

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Implement constitution checks in project templates