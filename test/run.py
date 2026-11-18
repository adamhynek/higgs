"""Build and run the portable production-core tests; Python standard library only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent


def find_cmake(explicit):
    if explicit:
        return str(Path(explicit).resolve())
    found = shutil.which("cmake")
    if found:
        return found
    if os.name == "nt":
        program_files = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)"))
        vswhere = program_files / "Microsoft Visual Studio/Installer/vswhere.exe"
        if vswhere.is_file():
            install = subprocess.check_output([
                str(vswhere), "-latest", "-products", "*", "-requires",
                "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
                "-property", "installationPath"], text=True).strip()
            bundled = Path(install) / "Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"
            if install and bundled.is_file():
                return str(bundled)
    raise RuntimeError("Install CMake >= 3.26, put it on PATH, or pass --cmake PATH")


def source_pins():
    cmake = (ROOT / "test/CMakeLists.txt").read_text(encoding="utf-8")
    block = re.search(r"set\(HIGGS_CORE_SOURCES\s+(.*?)\n\)", cmake, re.S)
    if block is None:
        raise RuntimeError("Cannot read the production source list")
    paths = [ROOT / "src" / name for name in block.group(1).split()]
    # Include the authored test package and a conservative header superset.
    paths += [p for p in (ROOT / "test").rglob("*")
              if p.is_file() and p.suffix in {".c", ".cpp", ".h", ".txt", ".py", ".md"}]
    paths += list((ROOT / "include").glob("*.h"))
    return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(set(paths))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, required=True,
                        help="Generated files and logs; must be outside this checkout")
    parser.add_argument("--report", type=Path,
                        help="Optional JSON result outside the checkout; defaults to BUILD/result.json")
    parser.add_argument("--cmake", help="CMake executable; PATH and Visual Studio discovery are tried by default")
    parser.add_argument("--generator", help="Optional CMake generator, e.g. Ninja or Visual Studio 17 2022")
    parser.add_argument("--config", default="Release")
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    build = args.build_dir.resolve()
    report = (args.report or build / "result.json").resolve()
    for output in (build, report):
        if output == ROOT or ROOT in output.parents:
            parser.error("Generated output must be outside this checkout")
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    build.mkdir(parents=True, exist_ok=True)
    report.parent.mkdir(parents=True, exist_ok=True)
    result = {"status": "failed", "qualification": "portable production core and linked C ABI only",
              "nativePluginBuilt": False, "gameStarts": 0, "commands": [], "sourceSha256": {}}
    started = time.monotonic()
    try:
        cmake = find_cmake(args.cmake)
        ctest = Path(cmake).with_name("ctest.exe" if os.name == "nt" else "ctest")
        if not ctest.is_file():
            raise RuntimeError("ctest must be installed beside cmake")
        pins = source_pins()
        result["sourceSha256"] = pins
        result["cmakeVersion"] = subprocess.check_output([cmake, "--version"], text=True).splitlines()[0]
        configure = [cmake, "-S", str(ROOT / "test"), "-B", str(build)]
        if args.generator:
            configure += ["-G", args.generator]
        generator = args.generator or os.environ.get("CMAKE_GENERATOR", "")
        if os.name == "nt" and (not generator or generator.startswith("Visual Studio")):
            configure += ["-A", "x64"]
        else:
            configure += ["-DCMAKE_BUILD_TYPE=" + args.config]
        commands = [
            ("configure", configure),
            ("build", [cmake, "--build", str(build), "--config", args.config,
                       "--parallel", str(args.jobs)]),
            ("ctest", [str(ctest), "--test-dir", str(build), "-C", args.config,
                       "--output-on-failure", "--verbose", "--no-tests=error"]),
        ]
        for name, command in commands:
            print("+ " + subprocess.list2cmdline(command), flush=True)
            completed = subprocess.run(command, cwd=build, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, errors="replace")
            log = build / (name + ".log")
            log.write_text(completed.stdout, encoding="utf-8")
            print(completed.stdout, end="", flush=True)
            result["commands"].append({"name": name, "argv": command,
                                       "returnCode": completed.returncode, "log": str(log)})
            if completed.returncode:
                raise RuntimeError(name + " failed; see " + str(log))
        if source_pins() != pins:
            raise RuntimeError("Source files changed during the test run")
        result["status"] = "passed"
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        result["error"] = str(error)
        print("ERROR: " + str(error), file=sys.stderr)
    finally:
        result["elapsedSeconds"] = round(time.monotonic() - started, 3)
        report.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print("Report: " + str(report), flush=True)
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
