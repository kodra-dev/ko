"""Plain user-editable instructions, keyed by workspace UUID rather than HIP name."""
from .checkpoints.storage import data_root

SHARED_DEFAULT = """# Shared Loom instructions

Work inside the current HIP by default. Experiment and validate there first.
Only consolidate work into ko when it is reusable and the user agrees.

You may read files anywhere to understand the task. Batch related reads when useful.
Keep file edits within the user's requested scope. Do not modify Loom's own code
from inside Houdini. These are working instructions, not an OS security boundary.

ko is a Git repository. Do not build a separate backup system for its source.
Do not commit or push unless asked.

Preserve useful context. Avoid repeated reads of unchanged content and aggressive
summarization. Inspect more when needed rather than guessing.
"""
HIP_DEFAULT = """# HIP instructions

Add guidance for this workspace here. These instructions override shared preferences.
They stay associated with the workspace when its HIP is renamed or saved under a new name.
"""


def instruction_paths(workspace):
    shared = data_root() / "LOOM.md"
    local = workspace.history.root / "instructions.md"
    for path, default in ((shared, SHARED_DEFAULT), (local, HIP_DEFAULT)):
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with path.open("x", encoding="utf-8", newline="\n") as stream:
                stream.write(default)
        except FileExistsError:
            pass
    return shared, local


def load_instructions(workspace):
    shared, local = instruction_paths(workspace)
    # Stable order/content; no timestamps or transient UI state in instructions.
    return ("User-editable working instructions. HIP preferences override shared preferences.\n\n"
            + "## Shared instructions\n" + shared.read_text(encoding="utf-8")
            + "\n## HIP instructions\n" + local.read_text(encoding="utf-8"))
