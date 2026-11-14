# PAID / REAL PROVIDER TESTS — explicit approval required

These tests use the official Codex process and Loom's existing ChatGPT login.
They consume subscription usage (or money/quota for a configured provider).
They are never run by the default offline suite. No token inspection/copying.

- provider_roundtrip.py: **one** model turn, one tiny fake tool, marker-only answer.
- scene_roundtrip.py: optional larger test; three turns and multiple real HOM tools.
- gui_startup_paid.py: optional larger test; two real turns in a disposable hidden GUI.

After obtaining explicit approval:
```powershell
./tests/loom/paid/run_paid.ps1 -ConfirmPaidUsage # minimal marker test only
./tests/loom/paid/run_paid.ps1 -ConfirmPaidUsage -Scene # opt-in fuller scene test
./scripts/loom_gui_test.ps1 -PaidProvider -ConfirmPaidUsage -TimeoutSeconds 210
```

For UI work use ../gui_startup.py and mock messages instead.
