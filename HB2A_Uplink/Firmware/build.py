"""Build the Keil project's C sources with GNU Arm Embedded, without an IDE."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET


def find_gcc():
    found = shutil.which("arm-none-eabi-gcc")
    if found:
        return found
    local = Path(os.environ.get("LOCALAPPDATA", ""))
    matches = sorted(local.glob("stm32cube/bundles/gnu-tools-for-stm32/*/bin/arm-none-eabi-gcc.exe"))
    if matches:
        return str(matches[-1])
    raise SystemExit("GNU Arm Embedded not found; pass --gcc PATH")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gcc")
    args = parser.parse_args()
    gcc = args.gcc or find_gcc()
    os.chdir(Path(__file__).resolve().parent)
    build = Path("build")
    build.mkdir(exist_ok=True)
    project = ET.parse("MDK-ARM/HB2A_Uplink.uvprojx")
    includes = project.find(".//Cads/VariousControls/IncludePath").text.split(";")
    includes = ["-I" + os.path.normpath(str(Path("MDK-ARM") / p)) for p in includes]
    sources = [os.path.normpath(str(Path("MDK-ARM") / f.text))
               for f in project.findall(".//Groups/Group/Files/File/FilePath") if f.text.endswith(".c")]
    flags = ["-mcpu=cortex-m3", "-mthumb", "-Os", "-g3", "-ffunction-sections",
             "-fdata-sections", "-fno-common", "-DUSE_HAL_DRIVER", "-DSTM32F103xB", *includes]
    objects = []
    for source in [*sources, "startup_stm32f103xb.s"]:
        obj = str(build / (Path(source).stem + ".o"))
        extra = ["-std=c11", "-Wall", "-Wextra", "-Wno-unused-parameter"] if source.endswith(".c") else []
        subprocess.run([gcc, *flags, *extra, "-c", source, "-o", obj], check=True)
        objects.append(obj)
    elf = str(build / "HB2A_Uplink.elf")
    subprocess.run([gcc, "-mcpu=cortex-m3", "-mthumb", "--specs=nano.specs", "--specs=nosys.specs",
                    "-Tstm32f103c8.ld", "-Wl,--gc-sections,-Map=build/HB2A_Uplink.map",
                    *objects, "-o", elf], check=True)
    suffix = ".exe" if str(gcc).endswith(".exe") else ""
    bin_dir = Path(gcc).resolve().parent
    for fmt, ext in (("ihex", "hex"), ("binary", "bin")):
        subprocess.run([str(bin_dir / ("arm-none-eabi-objcopy" + suffix)), "-O", fmt,
                        elf, str(build / ("HB2A_Uplink." + ext))], check=True)
    subprocess.run([str(bin_dir / ("arm-none-eabi-size" + suffix)), elf], check=True)
    print("Built Firmware/build/HB2A_Uplink.hex and .bin (flash at 0x08000000)")


if __name__ == "__main__":
    main()
