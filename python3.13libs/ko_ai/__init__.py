"""Compatibility entry point. New code should import loom."""
import importlib
import sys
import loom

for old, new in (("scene", "checkpoints.scene"), ("storage", "checkpoints.storage"),
                 ("tools", "tools"), ("provider", "provider"), ("panel", "panel")):
    module = importlib.import_module("loom." + new)
    sys.modules[__name__ + "." + old] = module
    # Preserve legacy attribute access used by callers and unittest.patch.
    if old in ("scene", "storage"):
        setattr(loom, old, module)
sys.modules[__name__] = loom
