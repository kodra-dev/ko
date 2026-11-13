# Ko AI — Product Goals

This document is the durable product brief. It records the intended experience and requirements, not a particular implementation. It supersedes reliance on the original planning conversation. Implementation choices and current progress belong in separate documents.

## 1. Vision

Make Houdini feel like a collaborative AI workspace, analogous to a code editor with an integrated agent pane.

The artist and assistant can converse continuously, explore ideas, inspect the scene, make changes, compare alternatives, and guide one another. The assistant accelerates procedural work without replacing artistic judgment or making the scene an opaque black box.

The commercial Houdini AI Assistant is a reference for useful benefits, not a specification to copy one-to-one. Prioritize a coherent, trustworthy experience over matching its branding, interface, or feature count.

## 2. Core experience

The user should be able to:

- Ask questions about Houdini or the current project.
- Have a back-and-forth conversation grounded in the open scene.
- Ask the assistant to explain a network, diagnose a problem, or recommend an approach.
- Ask for step-by-step guidance while doing the work manually.
- Direct the assistant to create or modify procedural work.
- Inspect what actually changed and why.
- Audition the result against earlier states before deciding whether to keep it.
- Explore multiple alternatives without losing previous work.
- Switch AI models without abandoning the conversation.

Guidance, discussion, investigation, and execution are equally legitimate workflows. A conversation must not imply permission to change the scene.

## 3. Intelligent scene navigation

The assistant should navigate the open HIP as a coding agent navigates a repository. The user should not have to select every relevant node or manually assemble the assistant's context.

### Context signals

Use the request and conversation together with relevant working context:

- Selected nodes and geometry.
- Current network editor paths and visible nodes.
- Nodes displayed in parameter panes, including pinned panes.
- Scene viewer context, displayed content, and selections.
- Current frame, camera, and relevant output state.
- Recent user and assistant changes where available.

These are clues to the user's focus, not strict inspection boundaries or definitive statements of intent.

### Autonomous investigation

The assistant should be able to:

- Survey scene structure and discover relevant networks.
- Search nodes, types, parameter values, and code.
- Follow upstream and downstream wiring.
- Trace channel references and other non-wire dependencies.
- Inspect accessible subnets, asset internals, materials, simulation sources, caches, and outputs.
- Expand its search when the initial context does not explain the problem.
- Inspect actual node availability and parameter schemas instead of inventing them.

Use progressive, targeted inspection rather than sending the entire HIP into every request. Explain important assumptions and ask for clarification when ambiguity materially affects the work.

**Broad discovery does not imply broad editing permission.**

## 4. Transparent, scoped changes

The user must be able to understand the assistant's work without relying solely on its conversational summary.

Present meaningful changes such as:

- Nodes created, deleted, renamed, or modified.
- Parameters and expressions before and after.
- Wiring changes.
- Text diffs for VEX, Python, and other code.
- Relevant flags, organization, dependencies, and output changes.
- Validation performed, failures, and remaining uncertainty.

Change records should reflect actual observed work, not merely the assistant's intended plan. Make it easy to locate affected content in Houdini.

Separate read-only assistance, proposed edits, and approved execution. Request explicit approval for risky, destructive, expensive, or externally consequential actions. Do not silently expand the task's scope.

## 5. Persistent versions and non-destructive exploration

Version navigation must feel like visiting saved states, not restoring a backup over the only copy of current work.

### Required guarantees

- Switching away from a modified scene preserves a recoverable version of the state being left, including manual changes.
- Switching to an older checkpoint does not erase newer checkpoints.
- Returning to the newer state is straightforward.
- Editing an older version creates an alternative branch rather than overwriting its descendants.
- Checkpoints remain available across tool reloads and application restarts.
- Preservation failure must abort a switch rather than risk losing the current scene.
- Corrupt or unavailable versions must be reported clearly.
- A failed checkout must provide a recovery route to the preserved state.

### Retention

**Delete checkpoints only through an explicit user cleanup action.**

No automatic expiry, pruning, startup cleanup, or silent deletion to satisfy a storage quota. Warn about storage pressure instead.

The default cleanup action means **keep the current scene and delete everything else**. It must not require selecting versions first. Preserve any uncheckpointed live edits before removing history, and always retain a verified recoverable copy of the current scene. Show an inline preview and explicit confirmation; clearly state that other branches, replaced revisions, Return targets, and other pinned versions will be removed. Cancelling or failing to preserve the current scene must delete nothing. Removing history must never make the retained scene unrecoverable.

### Updating an existing version

The user can explicitly replace a named version with the current live scene—for example, make new edits become the updated `Original` instead of requiring another version name. The normal version list should show the updated version. Keep its prior revision recoverable in a clearly separated revision history until the user explicitly cleans it up. Do not silently rewrite other versions or lose their original ancestry.

Switching between already saved, unchanged versions must not continuously create duplicate auto-preserved checkpoints. Automatic preservation is for live changes or states whose safety cannot be established, not for navigation itself.

### Navigation and identity

Provide clear labels for the active baseline, current manual edits, agent-delivered results, and alternative branches. Support useful version names and annotations, and quick return/A/B navigation.

Distinguish:

- Visiting or checking out a complete saved state.
- Reverting a particular task while preserving unrelated work.
- Comparing versions without replacing the current working state.

Do not present these as interchangeable actions.

## 6. Artistic comparison and auditioning

Undo is recovery. Comparison is artistic decision-making. Both are essential.

The artist may not know whether an assistant's result is an improvement until seeing it against the original. Acceptance must not be immediate or irreversible.

### Desired comparison capabilities

As appropriate to the work:

- Fast A/B switching.
- Side-by-side views.
- Wipe comparisons of aligned images/renders.
- Ghost or overlay views for geometry and trajectories.
- Synchronized playback for animation and simulations.
- Live inspection of candidate controls, alternate inputs, and camera angles.

Match camera, frame, lighting, and display settings by default where practical. Clearly label differences in comparison conditions.

Support both inexpensive captured previews and deeper live auditions. Full scene checkout should not be the only way to compare creative alternatives.

### Iteration

Natural requests should include:

- “Show the original again.”
- “Try a subtler version.”
- “Keep the clustering but restore the original scale distribution.”
- “Compare these two from the render camera.”
- “Keep both; I will decide later.”

Preserve the distinction between the original baseline, the assistant's delivered result, and subsequent user modifications. Do not silently discard user work when rejecting an earlier assistant task.

The assistant may describe visual differences and tradeoffs, but must not substitute its taste for the artist's judgment.

## 7. Reliable procedural assistance

Produce native, editable Houdini work that the artist can inspect and continue developing.

### Capability goals

- Scene explanations and contextual tutoring.
- Debugging and minimal, justified repairs.
- VEX, Python, expression, and automation assistance.
- Procedural network creation and refinement.
- Network organization, annotations, and technical documentation.
- Reusable HDA creation, meaningful controls, packaging, and testing.
- Materials, lighting, rendering, and Solaris/USD assistance.
- Simulation setup and diagnostics.
- Additional Houdini contexts as their workflows become reliable, including Copernicus, APEX, CHOPs, and PDG.
- Reference-assisted work using appropriate images, clips, documents, or existing projects.

Expand through useful, tested workflows rather than nominal support for every context.

### Verification

The assistant should inspect, plan, build, evaluate, repair within limits, and report evidence. Appropriate checks include structural validity, cooking, expected data, parameter behavior, temporal behavior, visual output, and dependencies.

A generated script is not a finished result. A successful cook is not proof of artistic or production quality.

For substantial creative work, prefer reviewable candidate results and controlled acceptance over immediate mutation of production content.

## 8. Collaboration, conflicts, and recovery

Users and agents may take turns modifying the same content.

- Detect when a proposal was based on a scene state that has since changed.
- Identify user edits and later dependencies before selectively reversing earlier work.
- Surface conflicts instead of silently overwriting newer decisions.
- Make partially completed or failed work visible and recoverable.
- Keep native Undo useful, without treating it as complete version control or a universal transaction.
- Make clear what is and is not included in a saved version.

External caches, asset definitions, textures, exports, and simulation state require explicit dependency/versioning policies. Avoid having alternatives silently overwrite shared outputs. Never claim a HIP-only snapshot protects all external effects.

## 9. AI backend and model choice

- Support **ChatGPT subscription access** as the AI backend.
- API-key providers and other providers are not required.
- Use supported authentication and integration mechanisms; do not depend on scraping or credential extraction.
- Let the user discover and choose models available to their account.
- Allow switching models between turns in an ongoing conversation.
- Preserve the user's preferred model where available and communicate when it is unavailable.
- Expose useful activity, interruption controls, and provider-limit/error information.
- Keep autonomous retries, context growth, and expensive work bounded.

## 10. Safety, privacy, and resource awareness

### Calm, inline feedback

Display recoverable errors and unsupported states clearly inside the affected pane or relevant UI. Do not show popup error alarms or raw Python tracebacks when an inline message is sensible. Technical details may be available on demand.

Simultaneous Ko AI panes are not required. If the user opens a second pane, show a short message that two panes are unsupported, with a Retry action after closing the other pane. Do not disrupt the existing pane or its conversation.

- Treat scene content and reference files as data, not trusted instructions.
- Keep permissions understandable and scoped to the user's task.
- Distinguish metadata inspection from operations that cook, render, simulate, or execute node code.
- Require appropriate consent and budgets for expensive work and external writes.
- Explain relevant data sharing with the AI provider.
- Protect credentials and avoid unintended inheritance of unrelated agent capabilities.
- Report uncertainty, unsupported operations, and validation gaps honestly.

Safety should support exploration without making ordinary collaboration unnecessarily cumbersome.

## 11. Optional character-motion workstream

The broader vision may include character-motion assistance comparable in benefit to the reference product:

- Text-to-motion.
- Editable multi-segment motion timelines.
- Pose, trajectory, hand, and foot constraints.
- Video-to-motion.
- Houdini-native import, retargeting, and cleanup workflows.
- Interactive motion direction and recording.
- Terrain adaptation and appropriately bounded obstacle guidance.

Treat this as a separate optional workstream. Reuse suitable existing models rather than assuming model training is part of the product. Evaluate model/dependency licenses, hardware requirements, and quality limitations independently.

It must not delay a useful core Houdini collaboration experience.

## 12. Development and testing expectations

- This `ko` Houdini package may serve as the tool's home where appropriate.
- Target the **latest installed Houdini version**. A multi-version compatibility/test matrix is not required unless explicitly requested later.
- Make development easy to iterate without restarting Houdini. Provide a straightforward reload/reopen workflow.
- Give the user short, concrete manual tests with clear expected outcomes.
- Maintain automated tests and a way for the development agent to exercise real Houdini behavior independently.
- Use disposable scenes/processes for destructive tests; do not endanger the artist's open work.
- Distinguish automated/offscreen checks, real provider tests, and desktop/manual validation. Do not claim tests that were not run.
- Keep progress and known limitations in separate implementation/status documentation.

Work may proceed in substantial increments. When pausing, leave a concrete, usable milestone the artist can test to judge whether the direction is right.

## 13. Prioritization and success criteria

Prefer, in order:

1. Trustworthy preservation and understandable changes.
2. Useful scene-aware conversation, navigation, and guidance.
3. Non-destructive creative comparison and iteration.
4. Reliable, editable procedural construction and verification.
5. Broader specialized workflows.

The tool succeeds when:

- The user does not have to manually collect all relevant scene context.
- Conversation can fluidly alternate between advice and directed work.
- The user can identify what changed and inspect its effects.
- Trying an idea does not put existing work at risk.
- Older and newer alternatives remain easy to revisit.
- Artistic acceptance is based on comparison, not pressure to trust the model.
- The user can change models without losing the collaborative workflow.
- Development can continue from this brief without reconstructing the original conversation.
