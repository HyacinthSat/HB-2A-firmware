"""Load GNU Radio DLLs from the active Windows conda environment."""
import os
import sys
from pathlib import Path

DLL_HANDLES = []
if os.name == "nt":
    prefix = Path(sys.executable).parent
    os.environ["PATH"] = str(prefix / "Library/bin") + os.pathsep + str(prefix) + os.pathsep + os.environ["PATH"]
    os.environ["CONDA_DLL_SEARCH_MODIFICATION_ENABLE"] = "1"
    if hasattr(os, "add_dll_directory") and (prefix / "Library/bin").exists():
        DLL_HANDLES.append(os.add_dll_directory(str(prefix / "Library/bin")))

