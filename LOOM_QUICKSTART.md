# Loom quick start

Current architecture, behavior, manual checks, migration, and milestones:
[LOOM_README.md](LOOM_README.md).

```python
import loom
from loom import checkpoints
checkpoints.open_pane()  # independent, no provider needed
loom.open_pane()
```

Use **Loom → Reload Loom + Checkpoints** for package-wide code reload.
Restart once after this naming migration; save the HIP/desktop to persist migrated identities.

- Data home: `%LOCALAPPDATA%/Loom`; `LOOM_HOME` overrides it.
- Installation helper: `scripts/loom_setup.ps1`.
- Offline checks: `scripts/loom_test.ps1`, `scripts/loom_gui_test.ps1`.
- Real-provider tests: **`tests/loom/paid/`** — explicit approval required; costs usage.
- **Conversations** opens the two-line manager: double-click to activate, F2 rename, Delete selected.
  New conversations request a short AI name as part of the first turn; manual names win.
  No archive state: all conversations are visible until deleted.
- Right-click a message: **Branch here** keeps the original chat, **Delete from here** removes
  that message and later ones, **Edit prompt** puts a user prompt back in the composer.
  Old/intermediate messages work too. Scene/files stay unchanged; nothing is sent automatically.
- **Edit scene** off: read-only answers/guidance. On: apply requested, checkpointed edits. No pending-review UI.
- **Context** inspects the current setup. The right-aligned `≈$0.12 C14%` shows estimated API-equivalent thread cost and current context fill; `—` means unavailable.
- Local help: Loom can search/read `H:/Docs/houdini_help`; override with `LOOM_HOUDINI_DOCS`.
