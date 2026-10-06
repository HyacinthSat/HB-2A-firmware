"""Build the flight codecs and the FFTW-backed native spectrum helper."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def build():
    compiler = shutil.which("gcc")
    if not compiler:
        raise RuntimeError("GCC is required once to build the shared flight codecs")
    destination = HERE / "build"
    destination.mkdir(exist_ok=True)
    output = destination / ("hb2a_wire.dll" if os.name == "nt" else "libhb2a_wire.so")
    dsp_output = destination / ("hb2a_dsp.dll" if os.name == "nt" else "libhb2a_dsp.so")
    core = ROOT / "HB2A_Balloon/Target/Core"
    sources = [core / "Src" / (name + ".c") for name in
               ("rs8", "gmsk_downlink", "high_speed_ssdv", "ssdv", "uplink_protocol")]
    wire_temp = destination / ("hb2a_wire.new.dll" if os.name == "nt" else "libhb2a_wire.new.so")
    wire_command = [compiler, "-std=c11", "-O3", "-shared", "-fPIC",
                    "-I" + str(core / "Inc"), str(HERE / "native.c"),
                    *map(str, sources), "-o", str(wire_temp)]
    conda_prefix = Path(sys.prefix)
    fft_include = conda_prefix / "Library" / "include"
    fft_lib = conda_prefix / "Library" / "lib"
    if (fft_include / "fftw3.h").exists() and (fft_lib / "fftw3f.lib").exists():
        dsp_command = [compiler, "-std=c11", "-O3", "-shared", "-fPIC",
                       "-I" + str(fft_include), str(HERE / "spectrum_native.c"),
                       "-L" + str(fft_lib), "-lfftw3f", "-o", str(dsp_output)]
    else:
        raise RuntimeError("FFTW3 single-precision development files are required")
    # Compile to a temporary path first. A running station may have the old
    # wire DLL locked on Windows; in that case keep it and replace it on the
    # next startup after the process has exited.
    subprocess.run(wire_command, check=True)
    try:
        os.replace(wire_temp, output)
    except PermissionError:
        print("wire DLL is in use; keeping it until the station exits")
        try:
            wire_temp.unlink()
        except OSError:
            pass
    subprocess.run(dsp_command, check=True)
    print(output)
    print(dsp_output)
    return output, dsp_output


if __name__ == "__main__":
    build()

