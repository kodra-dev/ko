# Loom — Houdini harness and independent Checkpoints

**Continuing in a new coding context?** Read [LOOM_HANDOFF.md](LOOM_HANDOFF.md)
for the user-verified milestone, remaining priorities, constraints, and test policy.

**Loom** is the package name, covering the Loom agent harness and independent
Checkpoints feature. This milestone separates scene history from the agent before
adding rewind and broader tools. Persisted workspace conversations are now implemented.

## Manual verification

Use a fresh Houdini session for this checkpoint-model and module-layout migration. No installer, package
configuration, preferences, or running artist session has been modified by the tests.
In the Python Shell:

```python
import loom
from loom import checkpoints
checkpoints.open_pane()  # works without the agent, Codex, or authentication
loom.open_pane()         # independent Loom conversation pane
```

If needed, first add `H:/Assets/Assets_3D/HoudiniScripts/ko/python3.13libs` to
`sys.path`. The shelf and top menu are **Loom**. The canonical native agent interface
is `loom_workspace` in `python_panels/loom.pypanel`. Old saved desktops can still load
through a hidden compatibility definition; opening/reloading Loom rebinds them to the
new interface. Save the desktop afterward to persist that change.

Houdini's top menu now has **Loom → Toggle Loom / Toggle Checkpoints / Reload Loom +
Checkpoints**, provided this development package root is on `HOUDINI_MENU_PATH`
(normally inherited from `HOUDINI_PATH`). Restart Houdini once to discover the new
`MainMenuCommon.xml` overlay. There is no ambiguous reload button inside the agent.
Package reload updates all loaded feature code, including checkpoint storage/scene/UI,
and rebuilds the open views without loading/saving the live HIP or creating a checkpoint.
Chat and checkpoint selection survive; an agent turn must be stopped first. The agent
reconnects automatically afterward. Syntax errors are checked before tearing down working
views. Closed features stay closed; Checkpoints-only use does not import/start the agent.
The Python equivalent is `loom.reload()`. Native **Reload Interface** only refreshes
the chosen pane; use the top-menu command for a package-wide code reload.
Menu XML layout changes themselves require another restart.

**Toggle Loom / Toggle Checkpoints** close an already-open floating pane, or reopen
it at its own last normal size/position. These independent app-wide preferences live
in `<data home>/pane-geometry.json`, survive reload/restart, and never move docked panes.
Programmatic `open_pane()` remains open-only; `toggle_pane()` is the menu toggle.

Try these in a disposable scene:

1. **New checkpoint** immediately saves with a local `YYYY-MM-DD HH:MM:SS` name.
   Select it and press **F2** while Checkpoints has focus. Enter accepts a rename;
   Escape cancels. No naming dialog appears during checkpoint creation.
2. Create a second checkpoint BEFORE making an alternative edit. Change the scene,
   then double-click the first: edits save INTO the checkpoint you leave, keeping
   its ID and name. No Return button, auto-preservation row, or hidden revision is
   created. Double-clicking the current checkpoint is a no-op, even with live edits.
   **New checkpoint** first updates the current checkpoint, then creates its child.
3. The strong background and bold text identify the live checkpoint; names have no
   current-state suffix. Selection uses a thin solid outline and light neutral tint;
   a selected current row has a brighter fill and strong white outline. A newly created
   checkpoint is automatically the sole selection and keyboard-current item.
   Columns are draggable/resizable with horizontal scrolling for deep trees;
   widths survive package reloads. **Last updated** uses local time; creation time is in the tooltip.
4. **Capture preview** is a visible TOP-BAR button, independent of selection. It
   saves the current checkpoint first, then captures the viewport (which may cook).
   It is disabled without a current checkpoint. Changed graph/frame is not an error.
   A failed capture leaves the successfully saved checkpoint intact.
5. RMB offers **Delete checkpoint** only for non-current checkpoints; current deletion
   is disabled. Deleting a parent keeps its children and moves them up one level.
   **Delete descendant checkpoints** deletes all children/grandchildren, keeping the selected
   parent. It is disabled if any descendant is current, and backend protection rechecks this.
   One selected item can be compared with the live scene; two can be compared together.
6. **⋯ → Delete all other checkpoints** acts immediately, with NO confirmation panel:
   save/verify the current checkpoint, then delete the others. If none is current,
   create one first. A failed save deletes nothing; locked deleted folders are
   reported and retried by a later explicit cleanup. External files/chat are untouched.
   The menu also retains access to legacy replaced revisions and manual refresh.
7. Open Loom alongside Checkpoints. Close/reopen either pane; the other must keep
   working. Use **Loom → Reload Loom + Checkpoints** and verify both panes rebuild. There is one shared workspace
   controller and OS lock. Scene-changing checkpoint actions wait for an active turn
   to finish; they never load a HIP underneath the agent.
8. Loom has no Changes/Versions tabs or pending-change review UI. With scene edits
   off it is read-only; enable **Edit scene** before requesting changes.
   Edits execute with checkpoints; old pending records are never auto-applied.
9. Use **Instructions → Open shared instructions / Open HIP instructions**. Edit
   the ordinary Markdown files directly. They are reread before every send. Rename
   the HIP or Save As: the same workspace instructions and checkpoint history remain.
   Save normally so the workspace UUID is persisted in the HIP.

### Persisted conversations

- **Conversations** opens a separate, modeless manager for the current UUID workspace.
  Each item shows its name and a faded first-user-message line. Active highlighting and
  selection outlines follow Checkpoints; selecting an item does not switch the chat.
  **Double-click / Enter** activates, **F2** renames the selected item, and **Delete** (button
  or key) removes it. **New conversation** creates/selects an empty one. Names need not be unique.
- Deleting a conversation removes its Loom record/history, not scene checkpoints, proposal
  evidence, or native provider files. Deleting the active conversation selects another; deleting
  the last creates an empty replacement. There is no archive/restore state; conversations
  hidden by older versions become ordinary, visible items without losing their history.
- During the first user turn, Loom asks the AI to choose a short name through the existing
  scene tool's reserved metadata query (`operation=context`, `query=@conversation-name:TITLE`).
  This is part of the ordinary turn, not a separate background prompt or provider thread, and
  works with Edit scene off. An explicit manual rename wins. Existing named/history-bearing
  conversations are not retitled. If the turn fails or the model omits naming, the initial
  name remains; no hidden retry/model call is made.
- Messages/tool details, native provider thread ID, model choice, draft, and usage stay with
  their conversation. Reload/reopen restores the selected conversation. Switching resumes
  its native thread on the next send; switching itself never calls a model or changes the scene.
- The last explicitly used model remains the app-wide default; each conversation remembers its
  own choice. An unavailable saved model requires an explicit available choice, not a silent
  substitution. **Edit scene** remains a workspace preference, not per conversation.
- Manager browsing stays available during a turn; mutating/activation actions are disabled
  or rejected until it finishes. **Tidy Network** creates its own
  conversation after validating scope/permission; your unsent draft stays in the original.
- `<workspace>/conversations.json` atomically stores records and selection. Existing `chat.json`
  imports as the initial conversation; old `conversations/*.json` imports as normal entries.
  Originals remain untouched, and migration is not repeated. Proposal/result files and native
  usage files remain in their existing locations; new proposals are tagged by conversation.
  Historical untagged proposals are available to the initial migrated conversation only.
- Manual check (no model calls needed): type a draft, create another conversation, open
  **Conversations**, select/F2 rename, double-click back, delete a disposable conversation,
  then reload/reopen. Check selection, messages, and drafts survive;
  checkpoint selection and the live scene must not change.

### Branching and editing history

Right-click a user or AI message:

- **Branch here** starts a new conversation, retaining the original. On an AI message it keeps
  history through that exact message, not later actions. On a user prompt it starts before the
  prompt and puts its text into the draft. Works for older and intermediate replies too.
- **Delete from here** removes the selected message and everything after it, keeping the same
  conversation/name and any current draft. It is also available on action/activity cards and their
  collapsed headers: deleting a grouped **3 actions** row starts at its first underlying action.
  Deleting only a later reply intentionally leaves earlier actions. Deleted content is excluded
  from model context; scene edits already made are not undone.
- **Edit prompt** (user messages) does the same removal and refills the composer with that prompt.
  Nothing is sent automatically. This replaces an unsent draft if present.

All three leave the scene, checkpoints and files unchanged; they are disabled during a turn.
Deleting history does not undo the cost of work already done. Deleted-tail proposal summaries
are excluded from subsequent context, while evidence/checkpoint files remain intact.

Implementation details:

- Recorded exact turn endpoints can use native `thread/fork` (`beforeTurnId` / `lastTurnId`),
  with goal auto-continuation deferred. No full-future fork hidden behind a trimmed UI.
- Old/intermediate messages, exact cuts before later tool activity, and offline branching use a
  fresh provider thread. The retained **visible transcript** is replayed once as historical data
  with the next user request; no model call occurs just to branch/delete/edit. Native hidden
  reasoning and old image bytes are not reconstructed. The model is told not to re-execute old
  requests/tool calls and to inspect the live scene as needed.
- Delete/edit also start a fresh provider context, not just a cosmetically shortened chat. Prefix
  replay survives reload, is consumed only after turn acceptance, and is not sent repeatedly.
  Native IDs from the discarded thread are not reused. Branching before a replay-containing
  first turn must retain that replayed prefix too.
- The original branch draft stays with the original; a new branch has its own model/provenance
  and usage. Same-conversation deletion carries forward already-reported cost, resetting only
  provider token counters/context-fill. Historical untracked cost remains unknown.
- Tests use mocks, never paid calls. Manual check: branch from an old/intermediate reply, then
  try Edit prompt/Delete from here and verify exact history, draft, unchanged scene and reload.
  Live model behavior with replay/bounded native forks still needs manual verification.

### Chat, input, local settings, and edit permission

- On Windows, a successful full AI turn plays Loom's own soft, rising three-note chime.
  Individual tool actions/messages never chime; failed, interrupted, stopped, idle/replayed
  or duplicate completion events do not notify. Playback is asynchronous and audio failures
  never fail the job. This is a custom generated WAV, distinct from pi's system Asterisk sound;
  it is cached under `<data home>/sounds/loom-complete-v1.wav`. Tests mock playback.
- One click activates the native pane and places the composer caret. **Enter sends**;
  **Shift+Enter inserts a newline** (including keypad Enter; IME composition is not submitted).
- Chat text/input is slightly larger. Message colors distinguish user/assistant; there are
  no You/Loom message headers. Copy/Details are floating icon buttons shown only on hover.
  The composer has no explanatory shortcut or permission-state lines. Tool activity stays
  collapsed, with real arguments/results in Details; failures expand automatically.
  Message Markdown cannot load local/network images or open links.
- **Tidy Network** in the Loom header sends an AI cleanup request (requires a connected model
  and **Edit scene**). Multiple selected nodes: only those; one selected: its upstream/downstream
  connected component in the same network; none: all direct children of the visible Network Editor.
  If multiple Network Editors are visible with no selection, select a node to disambiguate.
  Scope is frozen when clicked, a new conversation is selected, the unsent draft stays in the
  original conversation, and edits use the normal AI checkpoint.
  Cleanup asks for clearer layout, meaningful shorter names, behavior-preserving merge/split refactors,
  and comments/sticky notes/boxes only where difficult sections need explanation. Existing node comments
  are checked against the actual graph and corrected or removed when stale/incorrect. Layout uses explicit
  AI-chosen coordinates, not Houdini auto-layout: prefer tall top-to-bottom flow, semantic groupings and
  horizontal alignment of parallel roles where appropriate. Assume rounded wires and reserve 2.25 units
  per node/name plus column gaps; do not change the user's wiring-style preference. These are AI layout
  guidelines, not a guarantee of pixel-perfect label clearance. Risky changes should be skipped rather
  than breaking references. No automatic display-flag changes run for tidy turns.
- The old Ask/Propose modes are replaced by **Edit scene**, beside the composer.
- The composer row has **Context**, **Edit scene**, and a right-aligned `≈$0.12 C14%`.
  Cost is a standard API-equivalent estimate from reported usage deltas, using the model
  active for each call (including rerouting), cached-input rates, reported cache writes
  at the 30m rate, and long-context pricing where applicable. It is not subscription billing.
  C is the latest model-call token count divided by its reported context window, not
  lifetime cumulative tokens. Usage persists per thread through reloads/reconnects;
  new conversations reset it. Missing pricing, incomplete pre-tracking history, and
  unavailable context size show `—`, rather than a fabricated zero. Pricing is a local
  snapshot in `usage.py` (2026-11-14), not fetched during chat; hover for a short explanation.
  **Off (default):** read-only answers/guidance; edit tool calls are blocked, not queued.
  **On:** save the current checkpoint in place, then carry out edits in **one new AI child
  per turn**, including multi-proposal turns. No extra Before checkpoint, no AI: name prefix,
  and no second Apply confirmation. An initial baseline is created only if no current
  checkpoint exists. Older pending proposals are not auto-applied.
- Checkpoint creation accepts `workspace.checkpoint("Extrude faces", created_by="ai")`.
  AI titles are compact (at most 40 characters), with a small sparkle icon and tooltip,
  not an extra column. Origin metadata survives rename/save. Normal user labels are not capped.
- New nodes are placed near their connections with collision avoidance; pre-existing node
  positions stay unchanged by default. The final SOP gets a sensible display/render flag,
  preserving an already-displayed downstream OUT. Explicit flags win. Full layout requires
  an explicit `layout` step with `all=true`; normal layout targets only newly created nodes.
- Opening Loom automatically connects using the existing sign-in and last-used model;
  this does not send a prompt or start inference. Imports and Checkpoints-only use do
  nothing. Missing login/failure stays inline; browser sign-in remains a user action.
- **Edit scene** is remembered in the UUID workspace’s `preferences.json`.
  Reopening/reloading and Save As restore it; other HIP workspaces have separate preferences,
  defaulting off. The grant is frozen per turn, disabled during a turn,
  and revoked for the rest of the turn if automatic application fails. It does not
  authorize external file writes, simulation/render actions, or arbitrary execution.
- Local version, keymap, FPS and update mode accompany the turn. Shortcut questions use
  `loom_scene operation=context query=PolyExtrude` (or a key) to search live resolved
  Houdini bindings, including tool invocation and context-specific overrides. Missing
  matches are not proof that no custom shortcut exists.
- Local Houdini help is available through `loom_scene operation=docs_search` (query words,
  path `/` or a relative directory) and `operation=docs_read` (returned relative file path;
  offset/limit page lines). Uses `LOOM_HOUDINI_DOCS` if set, otherwise
  `H:/Docs/houdini_help`, then `$HFS/houdini/help`. Requires extracted text/HTML help;
  packaged help archives are not searched. Reads are root-confined and never execute examples.
  Search is lexical and bounded; narrow the directory/query when results are partial.
- `schema` with empty query inspects an existing node. With query set to an exact child
  type (e.g. `subdivide`), it reads that type's templates in the network at `path`, without
  creating a temporary node. Wrangle internal unconnected inputs no longer crash graph tracking.
- Structured `set` accepts scalar parameter names or tuple names with an array of exactly
  the tuple's component count (e.g. `dir: [0, 1, 0]`). Every component is checked for
  animation/expressions before any tuple component is changed.
- Geometry queries: empty returns the summary; `primitives` pages primitive numbers,
  point numbers, mean point positions, and normals; `groups` pages group names/types/counts;
  `group:primitive:NAME` and `group:point:NAME` page member numbers. Unsupported queries
  fail explicitly. Group patterns are not VEX and do not implicitly promote point `P`
  to primitive selection. Loom is instructed to verify nonempty selections and changed
  geometry rather than equating successful execution with a successful modeling result.
- Geometry inspection no longer prompts for every cook. A spinner indicates work, Stop
  cancels the agent/cooperative inspection, and native cooks use `hou.InterruptableOperation`
  (Esc). HOM stays on the UI thread: **an uncooperative C++/Python/plugin cook can still block
  until it yields or returns**; in-process forced cancellation/unsafe worker-thread HOM is
  not claimed. Geometry may cook upstream code; this is not a side-effect/security sandbox.
- `loom_scene operation=viewport` (legacy threads: `operation=context, query=@viewport`)
  captures the already displayed Scene Viewer and sends a bounded PNG as native `inputImage`,
  not a text-only filename. No render, forced cook, HIP save, or new checkpoint. Manual-update
  images can be stale and are labeled as such. Raw base64 is excluded from local chat logs.
- The provider's code-mode **host** must remain enabled for dynamic tool routing,
  even though shell/execution/patch tools remain disabled. Disabling it caused the
  reported false “scene connection unavailable” replies. Tool errors now appear in chat.

UI patterns: [Claude Code transcript/tool details](https://code.claude.com/docs/en/interactive-mode)
and [Codex app workflows](https://developers.openai.com/codex/app/features.md).

### Normal Houdini Save / Save As

With either pane open, a successful normal Save/Save As also updates the current
checkpoint, preserving its ID/name. It copies the already-saved working HIP on the
next UI tick rather than issuing another Houdini save (which would dirty the document).
Failed checkpoint synchronization is reported separately: the normal HIP save still
succeeded. Closing both panes removes the save callback; later switching still saves
live changes before loading another version.

Checkpoint switching/capture/new-version creation does NOT overwrite the working
HIP on disk. Switching preserves its current filename and `$HIP`, even after Save As;
it must not redirect the next Ctrl+S to a checkpoint's historical source filename.

Scene versions are editable, not immutable restore points. To retain an alternative,
explicitly create a new checkpoint BEFORE editing it. Updates use staged files plus
an atomic index publication; the previous files are removed only after a successful
commit. These staging files are not extra user-visible checkpoints or version history.
Old HIP/graph layouts remain readable. Proposal before/after IDs also refer to editable
versions; the recorded actual diff remains, but later edits can change those versions.

### Instructions and storage

The shared file is `<data home>/LOOM.md`; the per-HIP file is
`<data home>/workspaces/<UUID>/instructions.md`. Unsaved scenes use the same UUID
scheme; there are no filename-based sidecars to migrate. Shared instructions come
first, then HIP instructions. Existing files are never replaced with defaults.

The canonical data home is **`%LOCALAPPDATA%/Loom`**. On first use, the old
`ko-ai` folder is renamed as a whole, preserving login, preferences, instructions,
history and native conversation rollouts. Close old Loom/Houdini/private Codex processes
and restart once. Locked renames fail in place; two nonempty homes are never silently
merged. No credentials or opaque provider databases are parsed, copied or rewritten.
Relocated native rollout paths are supplied explicitly on conversation resume.

`LOOM_HOME` overrides the location; legacy `KO_AI_HOME` remains a compatibility reader
for explicit custom locations (these are not renamed). `LOOM_CODEX` selects the executable;
legacy `KO_AI_CODEX` is still accepted.

New HIP metadata uses `loom.workspace` / `loom.checkpoint`. Historical keys are translated
on binding without changing UUIDs; save the HIP normally to persist the new keys.
New native tools are `loom_scene` / `loom_propose`. Old provider conversations retain
their historical tool schemas, so hidden dispatch aliases keep them working without
rewriting conversation history or losing context. The same applies to legacy Python
imports and saved shelf-tool IDs; these are compatibility readers, not the package name.

Instructions are sent as provider developer instructions. Unchanged instructions
retain the native provider thread; changed instructions refresh it through resume.
Live scene/UI context is appended with the new user turn, not mixed into the stable
instruction prefix. Loom does not locally trim the transcript. Advanced context
budgets, cache statistics, and compaction controls are still future work.

A copied HIP or Save As retains its workspace UUID by default. Independent-copy
**Detach into new workspace** is not implemented yet. Workspace export is out of scope.
HIP snapshots still do not protect external scripts, caches, assets, or definitions.
Legacy pin metadata remains readable, but no longer blocks replacement or explicit deletion.

### Optional checkpoint actions

`loom.checkpoints.actions.register_action(id, label, callback, selection_count=1)`
registers a menu extension without opening a pane or loading an agent. A callback
receives `(workspace, checkpoint_ids, parent_widget)`. Registering the same ID replaces
it; `unregister_action(id)` removes it. Menus query the registry when opened, so no
pane/agent load ordering is required.

The real Loom **Suggest name** integration is not implemented in this milestone.
The extension seam and selection behavior are covered by tests; no placeholder AI
menu action pretends to perform inference.

## Layout and next milestones

Implementation now lives under `python3.13libs/loom/`. Checkpoint core, dialogs,
pane, action registry, and session service live under `loom/checkpoints/` and never
import the agent/provider. `python3.13libs/ko_ai/` is only a legacy import bridge.
Canonical scripts are `scripts/loom_setup.ps1`, `loom_test.ps1`, and `loom_gui_test.ps1`;
tests live under `tests/loom/`.

Milestones and remaining work, in small verifiable increments:

1. **Harness threads (implemented):** create/rename/switch/delete persisted conversations;
   retain model choices, drafts and tool activity. One active agent turn per Houdini session.
2. **History editing (implemented, pending manual verification):** Branch here at exact message
   cuts, Delete from here, and Edit prompt; bounded native forks or fresh-context replay. Explicit scene
   restoration remains unimplemented; editable checkpoint IDs alone do not guarantee the scene
   as it was at message time. Never silently undo ko source changes.
3. **Context management:** deterministic prefixes and tool schemas, append-only native
   thread history, on-demand retrieval, batched reads, conservative budget-triggered
   compaction with retained originals, usage/cache reporting where supported.
4. **Tools:** read files anywhere (multiple paths/ranges), search/find, and reviewed
   precise file edits including ko. Scope is governed by plain instructions, not a
   claimed sandbox. Work inside the HIP first; promote proven reusable work into ko
   only with user agreement. Never self-edit Loom inside Houdini. Use Git rather than
   adding a second ko backup system; don't commit/push unless asked.
5. **AI integration:** Suggest name as an optional registered checkpoint action; editable
   confirmation of the result. No inference on every checkpoint creation. Better
   per-message edit-result and before/after checkpoint links (not pending-review UI).
6. **Installation boundary:** move Loom + Checkpoints together into one independent
   Houdini package, separate from basic ko tools. The Python namespace/dependency
   split is done; the physical Houdini-package extraction is intentionally not yet done.
7. Explicit workspace detachment for independent copied HIPs. No export work for now.

## Tests

```powershell
./scripts/loom_test.ps1
./scripts/loom_gui_test.ps1          # hidden Win32 desktop by default
./tests/loom/gui_launcher_smoke.ps1  # hidden-launch/logging/timeout cleanup checks
# Only when explicitly debugging interactively (opens visible Houdini windows):
./scripts/loom_gui_test.ps1 -Visible
```

Graphical tests create a separate inactive desktop in the logged-in Windows session.
They never switch to it, use physical input, or silently fall back to visible mode.
Houdini verifies its actual GUI-thread desktop, and viewport capture is checked for
nonblank rendered pixels. `launcher.json` records launch mode, desktop, process ID,
exit code, and input desktop before/after. Screenshots, previews, results, stdout,
and stderr remain in the printed temporary directory even after test cleanup.
A kill-on-close Windows job owns only the disposable test process tree: timeout,
launch failure, and normal completion release its processes and desktop handles.
This avoids focus/window interruption, not CPU/GPU contention or a security sandbox.

The default suites are **OFFLINE** and block provider process startup. UI tests use
mock messages and FakeTransport, not inference. Tests use disposable scenes/processes;
the graphical test launches its own Houdini and checks both native panes, real F2
rename, actual viewport capture, native Save/Save As synchronization without dirtying
its saved document, shared controller ownership across package reloads, and binding recovery.
Package reload is verified through its registered native hotkey. Toggle commands use
their registered labels and actual XML script bodies, across real Houdini idle ticks,
to verify closing/reopening and exact independent geometry restoration. No real/global
keyboard input is sent.
A separate offscreen process tests package reloads, busy guards, preflight failures,
selection/chat/live-edit preservation, and persistent extension actions.
All real-provider tests are isolated under **`tests/loom/paid/`**, with cost warnings
and explicit approval/opt-in. The default paid smoke is one tiny fake-tool call and a
marker-only response; larger scene/native scenarios are separate options. See that
folder’s README and `tests/loom/AGENTS.md`. Never run paid tests automatically for UI work.

Before the naming migration, opt-in signed-in tests verified real tool routing, the live
Ko keymap’s Alt+E binding, automatic PolyExtrude creation/wiring/distance, proposal-only
non-mutation, resumed conversations, and actual before/after HIP contents. The renamed
paid entry points have not been rerun without renewed permission.
