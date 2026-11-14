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
- **Edit scene** off: read-only answers/guidance. On: apply requested, checkpointed edits. No pending-review UI.
- **Context** inspects the current setup. The right-aligned `≈$0.12 C14%` shows estimated API-equivalent thread cost and current context fill; `—` means unavailable.
- Local help: Loom can search/read `H:/Docs/houdini_help`; override with `LOOM_HOUDINI_DOCS`.
