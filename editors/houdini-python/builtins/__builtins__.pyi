# Names Houdini provides to Python code without an import. Shelf tools,
# parameter callbacks, HDA event handlers/modules and Python SOP code all run
# with hou imported, and most of them receive a kwargs dict.
from typing import Any

import hou as hou

kwargs: dict[str, Any]
