---

description: "Task list for Front Page Heading Update and Book Index Implementation feature"
---

# Tasks: Front Page Heading Update and Book Index Implementation

**Input**: Design documents from `/specs/010-front-page-heading-update/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- All paths are relative to repository root
- Docusaurus structure: `docs/`, `src/`, `static/`, config files at root

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Verify current front page content in docs/intro.md
- [X] T002 [P] Review existing sidebar configuration in sidebars.js
- [X] T003 [P] Document current navigation structure and content

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Create backup of current docs/intro.md file
- [X] T005 [P] Review docusaurus.config.js for current site metadata
- [X] T006 [P] Document the current book structure and modules
- [X] T007 Identify all references to the old front page heading
- [X] T008 Update docusaurus.config.js with correct book title metadata

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Updated Front Page Heading (Priority: P1) 🎯 MVP

**Goal**: Update the front page heading to display the book title "Physical AI & Humanoid Robotics"

**Independent Test**: Users can visit the front page and see the correct book title instead of the default "Tutorial Intro" text

### Implementation for User Story 1

- [X] T009 [P] [US1] Update docs/intro.md title to "Physical AI & Humanoid Robotics"
- [X] T010 [P] [US1] Update docs/intro.md H1 heading to book title
- [X] T011 [P] [US1] Update sidebar position for intro page in sidebars.js
- [X] T012 [US1] Adjust intro page content to reflect book purpose
- [X] T013 [US1] Test that front page displays correct heading
- [X] T014 [US1] Verify navigation from front page to other sections works

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Book Index Instead of 5min Tutorial (Priority: P2)

**Goal**: Replace the 5-minute tutorial content with a comprehensive book index

**Independent Test**: Site's navigation leads to a proper book index rather than a generic tutorial

### Implementation for User Story 2

- [X] T015 [P] [US2] Create new quickstart.md file with book index structure
- [X] T016 [P] [US2] Add all modules and chapters to the book index in quickstart.md
- [X] T017 [P] [US2] Update sidebar navigation to include the book index in sidebars.js
- [X] T018 [US2] Link from updated front page to the book index
- [X] T019 [US2] Update docusaurus.config.js navbar to reference book index
- [X] T020 [US2] Verify all book content is accessible through new index

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Consistent Navigation Experience (Priority: P3)

**Goal**: Maintain consistent navigation throughout the site between front page, index, and content sections

**Independent Test**: Navigating between front page, index, and various content sections maintains consistent navigation patterns

### Implementation for User Story 3

- [X] T021 [P] [US3] Update sidebar categories with consistent naming in sidebars.js
- [X] T022 [P] [US3] Ensure all module entries have proper navigation links in sidebars.js
- [X] T023 [P] [US3] Update footer links to match new book structure in docusaurus.config.js
- [X] T024 [US3] Test navigation flow from book index to all modules
- [X] T025 [US3] Test navigation flow from modules back to book index
- [X] T026 [US3] Verify consistent navigation appearance across all pages

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T027 [P] Test updated site in different browsers (Chrome, Firefox, Safari, Edge)
- [ ] T028 [P] Verify responsive design maintains navigation on mobile/tablet
- [X] T029 Update any remaining "tutorial" references to "book" or "course"
- [ ] T030 Run accessibility audit on updated navigation
- [ ] T031 Test all internal links work correctly after changes
- [ ] T032 [P] Take screenshots of key pages with new navigation for review
- [X] T033 Verify GitHub Pages build works with new content structure

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tasks for User Story 1 together:
Task: "Update docs/intro.md title to Physical AI & Humanoid Robotics"
Task: "Update docs/intro.md H1 heading to book title"
Task: "Update sidebar position for intro page in sidebars.js"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence