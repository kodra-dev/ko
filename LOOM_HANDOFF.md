# Loom — handoff for the next coding context

## Start here

The user has **roughly manually tested the current milestone and says it seems to work**.
Do not rebuild the completed Checkpoints/UI work. Read `AGENTS.md`, this file, and
`LOOM_README.md`, then agree on the next small, manually verifiable milestone.

Completed, user-verified behavior is documented in `LOOM_README.md`.
Conversation-only branching is implemented below and awaits manual verification.
Do not implement this whole roadmap in one pass.

Repo: `H:/Assets/Assets_3D/HoudiniScripts/ko`.
All development changes remain **uncommitted**. Do not commit/push unless requested.
`Ko.keymap2` contains an unrelated user change: preserve it.

## Remaining tasks, in priority order

### 1. Earlier-message branching / rewind — first increment implemented, pending manual verification

- User rejected verbose menu labels and disabled old-message branching. Menu is now **Branch
  here**, **Delete from here**, and **Edit prompt** (user messages). Details live in tooltips.
- Branch here keeps the original future, cutting exactly after an AI message or before a user
  prompt (prefilled draft). Legacy/intermediate replies work; no boundary-unavailable menu.
- Native exact endpoints use bounded `thread/fork`, deferring goal continuation. Other cuts
  start a fresh thread, replaying only the retained visible transcript once with the next send.
  No inference just to branch. Never include later actions or reconstruct opaque reasoning/images.
- Delete/Edit keep the same conversation ID/name but reset provider context too. Edit refills
  the prompt without sending; Delete preserves the current unsent draft. Reported cost survives.
  A new proposal context_epoch excludes deleted-tail evidence from recent_proposals.
- Prefix replay persists across reload and is consumed only after turn acceptance. Clear old
  native IDs; branching before the first replay-containing turn must replay again, not lose prefix.
- Delete from here is also available on action/activity cards, including collapsed headers.
  Grouped actions carry the first raw-message identity/index so deleting a displayed group removes
  all its actions and later messages, without accidentally cutting at a display index.
- All actions reject busy/stale menus and never restore/change scene, checkpoints or files.
  Subsequent turns warn that historical observations can be stale; request live inspection.
- Remaining: manual verification of live replay/bounded forks/history deletion and separate
  explicit scene restoration with a validated scene/checkpoint association.
- Checkpoints are editable versions: an ID alone does not guarantee its contents still
  match the scene when an earlier message was written. Design stale-context checks;
  do not advertise historical scene rewind without solving that association.
- Never silently roll back ko source files as part of a conversation rewind.

### 2. Context management and observability

- Stable/deterministic instruction and tool prefixes; append-only native conversation history.
- Keep volatile scene/UI state near the latest user input, not in stable instructions.
- On-demand, selective, batched retrieval instead of dumping/re-reading everything.
- Conservative budget-triggered compaction with originals retained; preserve useful context.
- Show usage/cache statistics where the provider supports them; make compaction observable.
- Avoid excessive tool-result payloads and unnecessary screenshots/model calls.

### 3. Tool development — first increment implemented, pending manual verification

- `loom_python` executes requested HOM/UI work on the UI thread in Scene Edit or Tool Dev.
- `loom_files` provides batched paged reads, immediate listing, literal in-file search, and
  hash-checked exact edits/new files; writes require Tool Dev. No pending-review popup.
- Remaining: recursive file search/find and richer file navigation; do not rebuild working tools.
- Precise writes/edits stay in the requested scope, including ko source when requested.
- Work in the HIP first; promote reusable work into ko only with user agreement.
- Scope/self-edit restrictions are working instructions, **not an OS security sandbox**.
- Do not let Loom edit its own running harness inside Houdini.
- Use Git for ko source history; do not add another source backup system.

### 4. Optional AI integration and review polish

- Implement the real Checkpoints RMB **Suggest name** action using the existing optional
  action registry, with an editable result. No model request on every checkpoint creation.
- Pending-change review was removed at user request. Edits off is strictly read-only;
  edits on applies with checkpoints. Keep checkpoint links useful without reintroducing review UI.
- Keep generated checkpoint names short, without AI:/Before: prefixes.

### 5. Physical package extraction

- Extract Loom + independent Checkpoints into one installable **Loom** Houdini package,
  separate from basic ko tools.
- Python namespaces and dependency separation are already implemented; physical package
  extraction is **not** done. Do not confuse the branding migration with installation work.
- Preserve migrations, compatibility readers, menu/shelf/panel definitions, and independent
  Checkpoints-only use.

### 6. Independent-copy workspace detachment

- Add an explicit detach/new-workspace operation for a copied HIP that should become independent.
- Current rename, Save As, and file copies retain the UUID/workspace by design.
- Workspace export is out of scope for now.

### 7. Truly isolated long-running inspection, if needed

- Geometry inspection is now prompt-free and has cooperative cancellation, **not a safe
  background HOM worker**. A non-yielding native/plugin/Python cook can still block the UI.
- If full responsiveness/forced cancellation becomes necessary, design an isolated process
  with explicit scene/dependency/cache semantics and licensing/cost considerations.
- Do not move arbitrary HOM/Qt calls to Python threads and claim they are safe.

## Completed behavior — preserve it

### Chat / agent

- Native Loom pane, separate from Checkpoints; one shared session controller/scene lock.
- Opening Loom auto-connects using the existing official sign-in and last-used model.
  This only initializes/authenticates/lists models; **it does not submit a prompt**.
- No Loom view means no auto-connect. Checkpoints-only use never loads/starts the agent.
- Compact three-stop permission selector shows only the current label: Read Only / Scene Edit /
  Tool Dev. Click dots or use arrow keys; per-dot hover descriptions. Disabled during a turn.
  `permission_mode` is persisted per UUID workspace in `preferences.json`, restored on
  reopen/reload/HIP switch and Save As. New workspaces default Read Only. Legacy true
  `allow_scene_edits` migrates to Scene Edit, never Tool Dev.
- Read Only blocks Python/edits, not queues them; file read/list/search remain available.
  Scene Edit adds structured edits and live Houdini Python/UI control. Tool Dev additionally
  authorizes persistent scripts/panels/ko source edits. Python has full in-process privileges:
  modes are authorization/instruction policy, NOT a security sandbox. Scene Edit's no-file-write
  rule and Python self-edit/credential/shell/network restrictions are instructions, not isolation.
- Python uses a fresh namespace (`hou` provided), bounded stdout/stderr, cooperative trace-based
  Stop plus native interrupt/Esc, and UI-thread enforcement. No unsafe worker HOM or forced cancel.
  It shares one per-turn AI child with structured edits, even for UI-only calls. Syntax errors run
  nothing. Ordinary Python exceptions/syntax errors return failed + retryable=true, allowing
  inspection and correction in the same turn after successful checkpoint persistence. Runtime errors
  may leave partial scene/UI/files changes; empty scene diff is NOT proof of no effects. Never blindly
  retry earlier actions. Stop/native interruption, HIP switches, checkpoint/storage/infrastructure
  failures, structured-edit failures and file-write failures still revoke all further changes for
  the turn. Positive Python recovery markers never override Stop or a revoked grant.
  UI/files are NOT checkpoint-reversible.
- File writes require a fresh SHA-256 (empty only for new files), exact unique/nonoverlapping edits,
  and staged atomic replacement. Max file 1 MB, bounded batches/output, partial/skipped work explicit.
  Known credential/Git locations and running-harness writes are rejected by the file tool. No
  extra ko source backup system. Scoped Tidy Network turns cannot use Python/source writes.
- Legacy two-tool threads can use scene context query @python:{label,code} / @files:{requests_json}
  (JSON objects). The panel normalizes these before grant checks; direct dispatch checks them too.
  Native resume supplies current instructions/tools; no inference is needed merely to reload.
- Local-help tools: `loom_scene` operations `docs_search` / `docs_read`, root-confined to
  `LOOM_HOUDINI_DOCS` or `H:/Docs/houdini_help` (fallback `$HFS/houdini/help`).
- Schema query=exact child type now reads that type's templates, not the parent object.
  Scene tracking tolerates unconnected indirect inputs inside wrangles; two-call regression covered.
  The grant is frozen per turn; structured application/infrastructure failure revokes it for the
  remainder of that turn. Ordinary Python code errors can be corrected (see retryable policy above).
- Slightly larger Markdown chat text; user/assistant colors replace role headers.
  Copy/Details are hover-only floating icon buttons. Tool activity is collapsed; errors expand.
- Single-click composer focus; Enter sends, Shift+Enter inserts newline; IME-safe.
  No redundant shortcut or permission-state explanatory lines.
- Geometry inspection no longer asks for approval repeatedly. Spinner and Stop are present;
  native cooks use `hou.InterruptableOperation`/Esc. Cancellation is cooperative (see above).
- `loom_scene operation=viewport` sends a real bounded PNG as native `inputImage`, not just
  a path. Old schemas can use `operation=context, query=@viewport`.
  It captures the currently displayed Scene Viewer without forcing a cook/render/save;
  Manual-update images may be stale. Base64 is not stored in local chat messages.
- Local settings and live resolved hotkeys are inspectable. In particular, the user's
  Ko keymap has Alt+E for PolyExtrude invocation; do not substitute stock shortcut advice.
- Shared `<data home>/LOOM.md` plus workspace `instructions.md` are ordinary editable
  Markdown, reread before sends and sent as developer instructions.

### Checkpoints / AI edits

- Editable scene versions, not immutable snapshots. Switching saves into the current
  version, retaining its ID/name; New checkpoint saves current then creates a child.
- No auto-preserve/Return versions. Visiting current is a no-op.
- Normal Save/Save As updates current using the already-saved HIP; no second save that
  dirties the working document. Switching preserves the working filename and `$HIP`.
- AI saves current in place and creates **one AI child per turn**, reusing it across
  successive proposals in that turn. No separate Before checkpoint. Only a scene with
  no current checkpoint needs an initial user baseline as well.
- `workspace.checkpoint("Extrude faces", created_by="ai")` records AI origin.
  Titles are compact (max 40 characters), prefixes stripped, small sparkle icon/tooltip;
  no big AI column. Origin survives saves/renames.
- New nodes are arranged near connections with collision avoidance; existing positions
  stay put by default. Sensible final SOP display/render flags; preserve a displayed
  downstream OUT and explicit flag choices. Whole-network layout requires explicit `all=true`.
- Strong current highlight; selected current has white outline; other outlines thinner.
  New checkpoints auto-select. Columns are resizable and horizontally scrollable.
- Preview capture is a top-bar action targeting current, independent of selection.
- RMB Delete checkpoint cannot delete current; children are reparented.
- RMB Delete descendant checkpoints keeps parent and is disabled/rejected if any descendant
  is current. Bulk deletion publishes metadata atomically before deleting files.
- Delete all other checkpoints saves/verifies current first, no confirmation; failed save deletes nothing.
- Optional action registry exists independently of the agent; real AI Suggest name does not yet.

### Menus / windows / naming

- Top menu: **Loom → Toggle Loom / Toggle Checkpoints / Reload Loom + Checkpoints**.
- Toggles close an existing floating pane or reopen it with its own last normal position/size.
  Stored app-wide in `pane-geometry.json`; docked panes are not moved/closed by toggles.
- Programmatic `open_pane()` remains idempotent/open-only; `toggle_pane()` implements toggling.
- Package reload safely rebuilds loaded features and open views, preserves chat/selection/
  column widths/live edits, and refuses during a running turn. It does not create checkpoints.
- Menu XML changes require a Houdini restart; Python implementation changes use package reload.
- Canonical namespace/scripts/tests/panes/shelf are Loom. Default data home is
  `%LOCALAPPDATA%/Loom`; old `%LOCALAPPDATA%/ko-ai` is renamed as a whole when safe.
  Active workspace locks or conflicting nonempty homes stop migration instead of merging.
- `LOOM_HOME` / `LOOM_CODEX` are canonical overrides; legacy readers remain.
- New HIP keys: `loom.workspace` / `loom.checkpoint`; legacy keys translate without changing UUIDs.
- Keep hidden compatibility for old HIPs/desktops/shelf IDs/Python imports/native tool schemas.
  New tools are `loom_scene` / `loom_propose`; old threads may still expose ko_ aliases.
- Provider rollouts may contain old absolute paths after a home move; resume supplies the
  relocated rollout path when available rather than rewriting opaque provider databases.

## Testing rules — especially important

Read root `AGENTS.md` and `tests/loom/AGENTS.md`.

```powershell
./scripts/loom_test.ps1
./scripts/loom_gui_test.ps1
./tests/loom/gui_launcher_smoke.ps1
```

- Default tests are **OFFLINE** and block real provider startup. UI uses mock messages/providers.
- All real provider tests live in **`tests/loom/paid/`**. They cost subscription quota/money.
  **Get explicit user approval for that run before using them.** Development approval or
  opening the app is not approval to spend on tests. Keep default paid smoke tiny.
- No further real-provider tests were run after the user raised this concern. Some earlier
  signed-in tests verified tool calls, live hotkey lookup, PolyExtrude edits and saved scene
  contents; later naming/UI changes have only offline/mock/manual coverage.
- Official Codex owns authentication under `<data home>/codex`; never inspect/copy tokens.
  Auto-connect must not auto-open a login browser or request inference.
- Keep `features.code_mode_host=true`: disabling this host caused the earlier broken tool
  connection. Shell/execution/patch tools remain disabled. Installed tested Codex: 0.168.0.
- GUI tests use a hidden Win32 desktop, never switch desktops or use physical input, and
  never fall back visibly. `-Visible` requires explicit permission. Test process trees
  are isolated/terminated by the launcher job, not by killing artist processes.
- Tested Houdini: `C:/Program Files/Side Effects Software/Houdini 22.0.466/bin/`.
- Latest full offline suite: **233 tests** (96 unit, 49 HOM, 84 offscreen UI, 4 package reload).
  Python permission checks, shared checkpoints, partial failure, trace restoration/SystemExit,
  cooperative Stop, HIP switches, file hashing/exact edits/atomic failure/partial batches,
  frozen grants and legacy routes are covered offline. Python missing-API/partial/syntax recovery
  is tested separately from fatal checkpoint failure, cancellation and workspace changes. Hidden GUI also writes a disposable
  module, loads it via loom_python on the UI thread and opens/verifies/closes a pinned native
  floating Parm pane. The recovery milestone's GUI passed on retry after missed native reload-hotkey
  dispatch. During the subsequent independent Toggle Parameters change, full offline Loom tests passed,
  but both Loom GUI regression attempts stopped at the known legacy-root isVisible startup assertion
  (gui_startup.py:189), before exercising the changes. Dedicated Parameters hidden GUI tests passed
  on both 22.0.429 and 22.0.466; do not conflate those results with a full Loom GUI pass.
  No paid-provider test was run. User's live log confirms Python execution; it also exposed a guessed
  nonexistent hou.Tool.setHotkeySymbol API and overly broad turn revocation, now fixed by the retryable
  Python-error policy above. End-to-end live recovery still awaits user verification.
  Hidden native GUI passed, including the concise message menu and Edit prompt without changing
  conversation identity and deleting a collapsed action group after deleting its reply. The native
  test needed one retry for the known invisible legacy-pane startup issue. Earlier startup failures
  exposed qtParentWindow=None; geometry tracking
  now attaches lazily on idle, and GUI failure cleanup tolerates a missing root. No visible fallback
  or paid tests.
- Replay, exact cuts, one-time acknowledgement, deletion/edit, cost carry-forward, proposal
  scope resets, and scene invariance use offline/mock coverage. No paid-provider tests ran.
- Native open/close needs a real Houdini idle boundary; nested Qt `qWait` alone can mislead.
  The GUI toggle test advances across timer ticks. Reload is tested through a native hotkey;
  toggle items use their actual XML script body plus registered label checks, since shortcut
  rebinding for those commands was unreliable in Houdini.

## Source map

- `python3.13libs/loom/panel.py`: agent/controller, auto-connect, workspace preferences, transport UI.
- `chat.py`: native message cards/composer/spinner and message branching menu.
- `conversations.py`: atomic workspace conversation records/selection, naming, deletion and legacy migration.
- `conversation_ui.py`: separate modeless two-line manager; shared Checkpoints selection styling.
- `tools.py`: scene/proposal schemas, inspection, per-turn edit checkpoint grouping.
- `mode_control.py`: compact three-stop permission selector and current-state label.
- `execution.py`: bounded output and cooperative UI-thread Python execution (not a sandbox).
- `files.py`: bounded text read/list/search and hash-checked atomic writes/exact edits.
- `inspection.py`: UI-thread geometry scope and viewport PNG capture.
- `network.py`: conservative arrangement/display defaults.
- `provider.py`: official app-server transport, tool-host flags, resume path handling.
- `instructions.py`, `paths.py`: editable guidance and data-home migration.
- `window_state.py`: native floating geometry tracking/restoration, independent of agent.
- `checkpoints/{scene,storage,service,panel,ui,actions}.py`: independent scene history and UI.
- `runtime.py`: coordinated package reload.
- `MainMenuCommon.xml`, `python_panels/loom*.pypanel`, `toolbar/loom.shelf`: Houdini integration.
- `LOOM_README.md`: current behavior, manual checks, architecture and roadmap.

## Suggested first prompt in the new context

> Read AGENTS.md, LOOM_HANDOFF.md and LOOM_README.md, and verify current code.
> Conversation-only branching is implemented; check whether it has passed manual verification.
> Do not implicitly restore scenes or files. Preserve history and independent Checkpoints.
> Do not run paid provider tests without asking me first.
