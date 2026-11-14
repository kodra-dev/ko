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

### 3. File tools

- Read anywhere, with batched files/ranges; search/list/find tools.
- Precise reviewed writes/edits in the requested scope, including ko source when requested.
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
- Allow scene edits is persisted per UUID workspace in its `preferences.json`, restored
  on reopen/reload/HIP switch, and retained across Save As. New workspaces default off.
- Off: read-only answers/guidance; edit calls are blocked, not queued. On: requested structured changes are automatically applied.
- Local-help tools: `loom_scene` operations `docs_search` / `docs_read`, root-confined to
  `LOOM_HOUDINI_DOCS` or `H:/Docs/houdini_help` (fallback `$HFS/houdini/help`).
- Schema query=exact child type now reads that type's templates, not the parent object.
  Scene tracking tolerates unconnected indirect inputs inside wrangles; two-call regression covered.
  The grant is frozen per turn; application failure revokes it for the remainder of that turn.
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
- Latest full offline suite: **197 tests** (78 unit, 43 HOM, 72 offscreen UI, 4 package reload).
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
