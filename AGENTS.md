# Coding-agent test policy

Loom's default tests are OFFLINE:
- scripts/loom_test.ps1
- scripts/loom_gui_test.ps1 (hidden desktop; UI uses mock messages)

**tests/loom/paid/** and **-PaidProvider** are REAL PROVIDER tests.
They consume subscription quota/money. Obtain explicit user approval before running
them; approval to develop Loom is not approval to spend on tests. The minimal paid
smoke is one small fake-tool call. Larger scene tests are separate opt-ins.

Do not use -Visible without permission. Never inspect or copy authentication tokens.
See tests/loom/AGENTS.md and tests/loom/paid/README.md.
