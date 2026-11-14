# Loom test policy for coding agents

- Default tests are OFFLINE. Use scripts/loom_test.ps1 and scripts/loom_gui_test.ps1.
- UI rendering uses mock messages and FakeTransport, never a model call.
- tests/loom/paid/ contains REAL PROVIDER tests: they cost money/subscription quota.
- Do not run paid tests without explicit user approval for that run.
- Keep the default paid smoke to one tiny tool call and a short marker response.
- Larger scene/native scenarios are separate explicit options, not default coverage.
- Graphical tests use the hidden desktop. Never choose -Visible without permission.
- Do not inspect, print, or copy authentication tokens.
