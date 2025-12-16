<!--
SYNC IMPACT REPORT
Version change: 0.1.0 → 1.0.0
Modified principles:
  - Technical accuracy (was PRINCIPLE_1_NAME)
  - Clarity for intermediate AI/robotics learners (was PRINCIPLE_2_NAME)
  - Reproducible code and pipelines (was PRINCIPLE_3_NAME)
  - No hallucinated APIs or robotics claims (was PRINCIPLE_4_NAME)
  - Spec-first, consistent structure across all chapters (was PRINCIPLE_5_NAME)
  - Verified architecture and implementation (was PRINCIPLE_6_NAME)

Added sections:
  - Standards and Requirements (was SECTION_2_NAME)
  - Book Requirements (was SECTION_3_NAME)
  - Governance section with specific rules for this project

Removed sections:
  - None

Templates requiring updates:
  - .specify/templates/plan-template.md: Constitution Check section may need project-specific updates (⚠ pending)
  - .specify/templates/spec-template.md: Requirements section aligns with constitution (✅ no changes needed)
  - .specify/templates/tasks-template.md: Task categorization reflects principles (✅ no changes needed)

Follow-up TODOs:
  - RATIFICATION_DATE: Original adoption date needs to be determined
-->

# Physical AI & Humanoid Robotics Book + Integrated RAG Chatbot Constitution

## Core Principles

### Technical accuracy
All content and code must maintain strict technical accuracy with ROS 2, Gazebo, Unity, Isaac, VLA, and other specified technologies.
**Rationale**: Ensuring that all technical information is accurate prevents confusion and mistakes that could derail learners and practitioners.

### Clarity for intermediate AI/robotics learners
All materials must be clear and accessible to intermediate AI/robotics learners, neither too basic nor overly advanced.
**Rationale**: Targeting the right audience level ensures maximum learning effectiveness while preventing frustration or disengagement.

### Reproducible code and pipelines
All code examples and development pipelines must be reproducible by users following the documentation.
**Rationale**: Reproducibility is fundamental to trust and learning; users should be able to follow along without encountering unexplained failures.

### No hallucinated APIs or robotics claims
No false or fabricated information about APIs or robotics capabilities; all claims must be backed by verified sources.
**Rationale**: Maintaining technical integrity preserves the book's credibility and helps readers build genuine understanding and skills.

### Spec-first, consistent structure across all chapters
All chapters must follow the spec-first methodology and maintain consistent structure as outlined in templates.
**Rationale**: Consistent structure enables easier navigation and comprehension while reducing cognitive load for readers.

### Verified architecture and implementation
All architecture diagrams, APIs, and pipelines must be verified for correctness and reflect real implementations.
**Rationale**: Accurate representations prevent misleading readers and ensure they develop correct mental models of the systems.

## Standards and Requirements
Sources: official docs or reputable robotics/AI references. Inline citations only (links or short refs). All code runnable on Ubuntu + ROS 2 Humble. All diagrams must reflect real ROS 2/Isaac/RAG architecture. Writing clarity: grade 9–12.

## Book Requirements
Docusaurus site deployable to GitHub Pages. Mandatory modules: ROS 2, Digital Twin, Isaac Sim, VLA, Capstone. Each chapter must include learning goals, explanations, tested code, diagrams, exercises.

## Governance
All PRs/reviews must verify compliance with technical accuracy and reproducibility requirements. Code examples must run as documented. All content must meet the writing clarity standards. Amendments require documentation of impact on existing content.

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date pending | **Last Amended**: 2025-12-15
