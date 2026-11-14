"""Installation discovery without importing hou, running Houdini, or evaluating packages."""
from pathlib import Path
import os
import re


def _path(value, root):
    path = Path(os.path.expandvars(value)).expanduser()
    return (root / path).resolve() if not path.is_absolute() else path.resolve()


def find_houdini(settings, root, environ=None, platform=None):
    environ = os.environ if environ is None else environ
    platform = os.name if platform is None else platform
    root = Path(root)
    explicit = settings.get('houdiniPath')
    if explicit:
        # Invalid explicit paths are not silently replaced with a different version.
        return _path(explicit, root)
    if settings.get('discoverHoudini', True) is False:
        return None
    compiler = 'vcc.exe' if platform == 'nt' else 'vcc'
    for key in ('KO_VEX_HOUDINI', 'HFS'):
        value = environ.get(key)
        if value:
            candidate = _path(value, root)
            if (candidate / 'bin' / compiler).is_file():
                return candidate
    candidates = []
    if platform == 'nt':
        for base in dict.fromkeys(environ.get(key) for key in ('ProgramW6432', 'ProgramFiles') if environ.get(key)):
            candidates.extend((Path(base) / 'Side Effects Software').glob('Houdini *'))
    else:
        candidates.extend(Path('/opt').glob('hfs*'))
        candidates.extend(Path('/Applications/Houdini').glob('Houdini*/Frameworks/Houdini.framework/Versions/Current/Resources'))
    def version(path):
        matches = re.findall(r'(\d+)\.(\d+)(?:\.(\d+))?', str(path))
        return tuple(int(n or 0) for n in matches[-1]) if matches else (0, 0, 0)
    valid = [p for p in candidates if (p / 'bin' / compiler).is_file()]
    return max(valid, key=lambda p: (version(p), str(p)), default=None)
