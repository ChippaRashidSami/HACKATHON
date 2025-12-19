---

description: "Task list for Docusaurus UI Upgrade feature implementation"
---

# Tasks: Docusaurus UI Upgrade for Book Frontend

**Input**: Design documents from `/specs/008-docusaurus-ui-upgrade/`
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

- [ ] T001 Install Docusaurus dependencies using npm
- [ ] T002 Verify Docusaurus installation by running local development server
- [ ] T003 [P] Configure docusaurus.config.js with site metadata
- [ ] T004 [P] Configure sidebars.js with navigation structure

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 Create src/css/custom.css for consistent theme styling
- [ ] T006 [P] Create module directories: docs/module-3-isaac and docs/module-4-vla
- [ ] T007 [P] Configure GitHub Pages deployment settings in docusaurus.config.js
- [ ] T008 Set up consistent navigation in navbar configuration
- [ ] T009 Verify responsive design framework is properly configured

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Module-Based Navigation (Priority: P1) 🎯 MVP

**Goal**: Implement clear module- and chapter-based navigation via sidebar so readers can find and access content efficiently

**Independent Test**: Readers can navigate directly from the sidebar to any module or chapter without difficulty

### Implementation for User Story 1

- [ ] T010 [P] [US1] Create _category_.json for module-3-isaac with proper configuration
- [ ] T011 [P] [US1] Create _category_.json for module-4-vla with proper configuration
- [ ] T012 [P] [US1] Update sidebar configuration to include Isaac module items
- [ ] T013 [P] [US1] Update sidebar configuration to include VLA module items
- [ ] T014 [US1] Verify all modules and chapters are accessible via sidebar navigation
- [ ] T015 [US1] Test navigation flow between different modules and chapters

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Responsive and Accessible UI (Priority: P2)

**Goal**: Ensure the book frontend is responsive and accessible across desktop and mobile devices

**Independent Test**: The UI renders correctly and remains usable on different screen sizes and with accessibility tools

### Implementation for User Story 2

- [ ] T016 [P] [US2] Test current layout on mobile screen sizes (320px, 768px)
- [ ] T017 [P] [US2] Test current layout on tablet screen sizes (768px, 1024px)
- [ ] T018 [US2] Adjust CSS for responsive sidebar navigation on mobile devices
- [ ] T019 [US2] Implement proper heading hierarchy for accessibility
- [ ] T020 [US2] Add ARIA labels to navigation elements
- [ ] T021 [US2] Test accessibility with screen reader tools

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Consistent Theming and Typography (Priority: P3)

**Goal**: Apply consistent theming and typography throughout the book for a cohesive visual experience

**Independent Test**: All pages use the same fonts, colors, and styling consistently

### Implementation for User Story 3

- [ ] T022 [P] [US3] Define primary color palette in src/css/custom.css
- [ ] T023 [P] [US3] Define typography scale (font sizes, weights) in src/css/custom.css
- [ ] T024 [US3] Apply consistent styling to all module category pages
- [ ] T025 [US3] Apply consistent styling to all chapter pages
- [ ] T026 [US3] Verify consistent styling across all modules
- [ ] T027 [US3] Test typography readability across different devices

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T028 [P] Update intro.md with proper navigation links to modules
- [ ] T029 [P] Add favicon and social card images to static/img/
- [ ] T030 Update footer with proper module navigation links
- [ ] T031 [P] Add search functionality configuration to docusaurus.config.js
- [ ] T032 Verify GitHub Pages build works correctly
- [ ] T033 Run accessibility audit on the complete site
- [ ] T034 Document the new structure in quickstart.md

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
Task: "Create _category_.json for module-3-isaac with proper configuration"
Task: "Create _category_.json for module-4-vla with proper configuration"
Task: "Update sidebar configuration to include Isaac module items"
Task: "Update sidebar configuration to include VLA module items"
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