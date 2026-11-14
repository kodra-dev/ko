# Loom goals

The current plan and implemented behavior are maintained in
[LOOM_README.md](LOOM_README.md).

Loom is the package name; Checkpoints works independently of the agent/provider.
Persisted conversations and conversation-only branching are implemented; branching awaits
manual verification. Upcoming milestones: safe explicit scene rewind, conservative
cache-aware context management, file tools, optional AI checkpoint naming, and physical
extraction into a separate Houdini package.

Work inside the HIP first; promote reusable tools into ko only with agreement.
No commits or pushes unless requested. Real-provider tests require explicit approval;
ordinary UI/storage checks remain offline.
