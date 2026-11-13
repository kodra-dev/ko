# Ko AI — first working milestone

A reloadable Houdini 22 / Python 3.13 / PySide6 agent workspace in this existing `ko` package. Windows is the tested platform. Experimental: start in a disposable scene.

## Open it now (no Houdini restart)

In Houdini's **Python Shell**, run:

```python
import ko_ai
ko_ai.open_pane()
```

This registers `python_panels/ko_ai.pypanel` in the running Houdini session and opens a native Python Panel. You can also choose **Ko AI** from a Python Panel's interface menu after registration. Only one Ko AI pane is supported at a time. A second pane displays a short inline explanation and **Retry**, not a Python exception or popup. Close the first pane, then Retry to use the second one.

For a simpler standalone window:

```python
import ko_ai
ko_ai.show()
```

If this package's Python directory isn't already on `sys.path` in your running Houdini session:

```python
import sys
sys.path.insert(0, r"H:/Assets/Assets_3D/HoudiniScripts/ko/python3.13libs")
import ko_ai
ko_ai.open_pane()
```

**Reload tool** reloads the implementation and restores saved chat/proposals/history without restarting Houdini. Active native interface definitions are updated in place, not reinstalled: reinstalling a `.pypanel` can invalidate Houdini's binding and show a generic script error with an empty traceback. Reload can also repair a previously unbound Ko AI pane. Stop the active agent turn first. For an embedded panel, Houdini's **Reload Interface** also works. The `Ko AI / Reload` shelf tool is an additional entry point once Houdini discovers the new shelf file.

## ChatGPT subscription setup

The tested official Codex CLI is installed privately at `%LOCALAPPDATA%/ko-ai/tools`, not into this repository or global npm packages. It is already installed on the current development machine. To install/reproduce:

```powershell
./scripts/ko_ai_setup.ps1
```

1. Click **Connect**.
2. Click **Sign in with ChatGPT** and finish OpenAI's browser login.
3. Select a model using the **Model** picker. Models come from the account's `model/list`; no one-model hardcoding. Switching applies to the next turn, including within an existing conversation. Your selected model is remembered across reloads.
4. Send a message.

Only ChatGPT-managed authentication is accepted; there is no API-key fallback. Subscription eligibility, model availability, and usage limits are controlled by OpenAI.

Ko AI uses the official `codex app-server` JSON-RPC interface, not browser scraping or unofficial ChatGPT endpoints. Codex owns credentials and refreshes. Ko AI uses a separate `%LOCALAPPDATA%/ko-ai/codex` CODEX_HOME, so you may need to sign in even if ordinary Codex is already logged in. It neither reads/copies ordinary Codex tokens nor inherits your normal Codex MCP servers/config. Shell/code-mode/apply-patch tools are disabled through launch configuration; requests for command/file-edit approval are declined. This is not a security sandbox for Houdini node code.

The dynamic-tool API is experimental. The setup script pins the protocol version tested here: Codex CLI **0.168.0**.

## What works

- Native Qt conversation pane; streaming messages, stop, new conversation, persisted transcript.
- Live model discovery and model switching between turns; saved selection.
- **Ask / Guide** and **Propose edits** modes. No automatic application of proposals.
- Whole-scene discovery: network listing, recursive search through names/types/raw parameter values, node type discovery, parameters, connections, parameter schemas.
- UI context from visible current network/parameter/scene panes, including pinned-node hints, visible-node positions, frame/camera, and selection. Selection is not a navigation boundary.
- Geometry summary inspection after explicit cook approval.
- Reviewed node creation, scalar parameter edits (including VEX snippet text), wiring, flags, comments, layout, and deletion.
- Before/after HIP checkpoints and actual graph diffs for applied proposals.
- Stale-proposal rejection if tracked scene state has changed since the proposal.
- Full-scene checkpoints, branching history, visit/return without unchanged-navigation duplicates, explicit cleanup, pinning, renaming.
- **Replace selected** updates a named version from the current live scene; its previous snapshot stays recoverable under **Show replaced revisions**.
- Optional viewport stills and a side-by-side captured-preview comparison, plus scene/text/VEX diffs.
- Reload without restarting Houdini.

## Five-minute manual test: versions without AI

1. Make a Geometry object containing a Box SOP. Open Ko AI → **Versions**.
2. **Checkpoint now**, name it `Original`.
3. Change the box's X size to 3. Checkpoint it as `Wide`.
4. Change X size again to 7, **without checkpointing**.
5. Select `Original` and click **Visit selected** (or double-click it).
   - You should see the original box.
   - A new `Auto-preserved before visiting…` checkpoint should contain your X=7 state.
6. Click **Return**. The box should return to X=7.
7. Visit `Original`, change Y size, and make `Tall alternative`. It should branch from `Original`, not replace `Wide`.
8. Select two versions and **Compare**; inspect parameter and wiring diffs. Select one to compare with the current live graph.
9. Click **Reload tool**. Checkpoints remain.
10. Click **Clean up…** without selecting anything. It should show an inline plan to keep the current live scene and delete all other versions. **Cancel** must leave everything untouched; **Delete other versions** explicitly removes old branches, replaced revisions, Return targets, and other pinned versions. Uncheckpointed edits are saved first.
11. Repeatedly visit `Original` and `Wide` without editing: the checkpoint count should stop growing. Visiting the active unchanged version is a no-op.

### Update Original instead of making another named version

1. Make manual changes to the scene.
2. Select `Original` in Versions (select it; do not Visit it, which would load the old content).
3. Click **Replace selected** and confirm.
4. The visible `Original` now contains your current scene. Other versions are untouched.
5. Enable **Show replaced revisions** to visit or compare the previous `Original`. Its HIP has not been deleted. Existing descendants retain their actual old ancestry.

Pinned versions must be unpinned before replacing. Replacing an unchanged active version does nothing. Historical snapshots are kept until explicit cleanup.

For image comparison, enable **Capture viewport preview when checkpointing**, and create two versions using the same camera/frame/display settings. Capturing may cook geometry. Captured stills are not live geometry views; advanced live A/B/wipe and simulation playback are not implemented yet.

### Clean up everything except the current scene

No selection is required. **Clean up…** opens an inline summary and list of versions to remove. Only **Delete other versions** performs the cleanup. It keeps the current scene, including uncheckpointed edits, and removes every other checkpoint (including hidden revisions and pinned alternatives). The retained version keeps the current name. Return history is cleared; chat and external assets are untouched. Cancelling or failing to save the current scene deletes nothing. If files are locked, the pane reports it; a later explicit cleanup retries them.

### Duplicate panes and error messages

Open a second Ko AI Python Panel while the first is open. Expect only: “Ko AI doesn’t support two panes at once. Close the other Ko AI pane, then click Retry.” The original pane must keep working. Close it and click **Retry** in the second pane; the workspace and conversation should open there. Routine action errors appear in the pane's status area, with optional **Error details**, not popup alarms. Explicit action/permission confirmations are distinct from error alerts.

## Manual test: agent

After sign-in, start in **Ask / Guide**:

> Explore this HIP and explain how the displayed geometry is produced. Follow relevant dependencies even outside the selected network. Do not change anything.

Use **Inspect context** to see the initial hints. Tool entries in the transcript show autonomous inspection.

Then select **Propose edits**:

> Propose creating a new geo object called ko_ai_test with a box, a transform making it twice as tall, and an OUT null. Use a new network; do not modify my existing objects. Inspect available node types/parameters as needed.

Review **Changes**, then click **Apply reviewed proposal**. Application is synchronous in Houdini, creates before/after versions, and records the actual diff. Ask the agent to inspect the result afterward; allow geometry inspection if desired. Initial application itself does not automatically cook/verify every result.

Try changing the scene after a proposal but before Apply: it should refuse the stale proposal and ask for a fresh one.

Switch models, then ask a follow-up. The next `turn/start` explicitly carries the selected model. Reload/reconnect and check that the selection is remembered.

## Checkpoint semantics / safety

- Visiting another version reuses the current saved version when unchanged. Live edits get an auto-preserved HIP first. If preservation fails, switching is aborted.
- Change detection uses Houdini's scene modification token and timeline state, not only our incomplete graph diff or the unreliable dirty flag. Baselines survive tool reloads. If no trustworthy baseline exists (for example the first switch after upgrading from an older implementation or reopening an unmanaged scene), one conservative safety snapshot may be made. Subsequent unchanged visits reuse saved versions.
- **Replace selected** creates a new revision of the selected name without deleting the old snapshot. Replaced snapshots are hidden by default, accessible through **Show replaced revisions**, and removable only through explicit cleanup.
- Visiting the active base also preserves current manual edits before reloading that base.
- The state you leave remains selectable. Editing an older version creates a branch on the next checkpoint.
- No automatic pruning, expiry, startup cleanup, or hidden quota-based deletion. Storage can grow significantly; cleanup is yours to initiate. The explicit cleanup action retains only the current scene, including saving live edits first; its inline confirmation explicitly includes other pinned versions and hidden revisions.
- Each snapshot has its own HIP and SHA-256 integrity check; descendants do not need ancestor HIP files. An index-write failure can recover committed snapshot metadata on reopening.
- A workspace OS lock prevents two sessions from simultaneously writing its history.
- Checkpointing uses Houdini `mwrite -n` to save a copy without changing the working filename or `$HIP`. Checkout restores the checkpoint's logical source filename after loading. A normal Ctrl+S still saves to that working filename.
- Root-node user data stores workspace/checkpoint identity. Save your working HIP normally if you want reopening that file to retain its workspace association. Unsaved scenes remain recoverable from their workspace's checkpoint HIP files even after closing Houdini.
- Loading a HIP resets native undo history and may execute ordinary Houdini load scripts/callbacks. It may take time and recook. `$HIP` is restored after loading; scripts that run during load may briefly observe the checkpoint directory. Use absolute paths/`$JOB` for critical load-time dependencies for now.
- HIP checkpoints do **not** freeze external HDAs, caches, images, exports, or simulation RAM. Existing references can resolve to newer files. Do not let experimental alternatives overwrite shared outputs.
- Node creation/parameter changes can invoke node code or callbacks. Explicit review is required, but this is not an execution sandbox. Animated/expression parameters and parameter buttons are deliberately not edited by this first implementation.
- A failed edit batch may leave partial changes: these are recorded and checkpointed where possible, with a baseline available to visit. Undo grouping is not advertised as a transaction.
- The semantic diff is not a lossless HIP serialization. It tracks node types, raw parameter/keyframe data, inputs, layout, color, comments, and common flags. Embedded geometry, all specialized node state, external definitions, network boxes, and some UI state are not represented. Proposals refuse scenes beyond the 20,000-node tracking limit.
- Scene search/context sent in chat goes to OpenAI. Checkpoint files remain local. Transcripts contain scene data and are stored locally as plaintext; treat them as project-sensitive.

## Automated tests

```powershell
./scripts/ko_ai_test.ps1
# Optional exact Houdini version:
./scripts/ko_ai_test.ps1 -Hython 'C:/Program Files/Side Effects Software/Houdini 22.0.429/bin/hython.exe'
# After ChatGPT login, include an actual small inference/tool-call round trip:
./scripts/ko_ai_test.ps1 -Chat
# Actual desktop-pane lifecycle regression in its OWN graphical Houdini process:
./scripts/ko_ai_gui_test.ps1
```

The test runner targets the latest installed Houdini by default; testing a multi-version matrix is not required. These run in separate disposable processes, not your existing scene:

- Pure-Python history, corruption, locking, cleanup, atomic-write, and diff tests.
- Real-HOM save/load, branch/return, manual-edit preservation, save/load failure recovery, proposals, stale edits, permissions, geometry, and wiring tests.
- Offscreen Qt transcript, tools, model switching/persistence, comparison, and reload tests.
- Real official Codex initialize/account/model-list/thread-start protocol checks, including the dynamic-tool schemas.

The default protocol check makes **no model inference request**. Full inference is explicitly tested with `-Chat` after login.

`ko_ai_gui_test.ps1` launches and closes its own disposable graphical Houdini with isolated preferences and checkpoint storage. It checks actual Qt parenting/visibility, active-interface binding, Reload tool clicks, direct/native reloads, persisted chat/history, duplicate-pane messaging, and recovery after deliberately invalidating a binding. It never connects to or closes your existing Houdini process. Logs, result JSON, and a screenshot remain in the printed temporary directory. Native GUI operations run from a Qt timer; Houdini idle callbacks alone can produce misleading UI test results. Viewport flipbook capture and broader artistic workflows still need manual testing.

## Storage / development layout

Default home: `%LOCALAPPDATA%/ko-ai`. Override before opening Houdini with `KO_AI_HOME`. `KO_AI_CODEX` can point directly at a compatible native Codex executable.

```text
python3.13libs/ko_ai/
  storage.py    Atomic metadata, immutable history, integrity checks, graph diffs
  scene.py      Main-thread HOM adapter, UI context, checkpoint checkout/recovery
  tools.py      Read tools and reviewable structured edit proposals
  provider.py   Official Codex app-server transport / subscription integration
  panel.py      Conversation / Changes / Versions UI
python_panels/ko_ai.pypanel
scripts/ko_ai_setup.ps1
scripts/ko_ai_test.ps1
scripts/ko_ai_gui_test.ps1
tests/ko_ai/
```

## Next milestones (not claimed implemented)

1. Complete real-account end-to-end testing after sign-in; improve desktop pane/context/capture behavior from artist feedback.
2. Better visual change cards, node framing, per-parameter diffs and proposal previews; richer schema/docs lookup.
3. Persistent view/camera comparison state, matched-camera captures, fast A/B auditions without a full scene reload.
4. Isolated candidate construction and bounded verified cook/repair loops.
5. Scoped semantic reverts with conflict detection, beyond full-checkpoint checkout.
6. HDA packaging/testing, external dependency versioning, cache isolation, simulation/render budgets.
7. Optional character-motion module.
