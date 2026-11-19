"""Prove the unresolved-instance test detects a mutation in a new external copy."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

sys.dont_write_bytecode = True
import run as runner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True, type=Path,
                        help="External output directory; its source/ and build/ must not exist")
    parser.add_argument("--cmake", help="Optional CMake executable")
    parser.add_argument("--generator", help="Optional installed CMake generator")
    args = parser.parse_args()
    out = args.build_dir.resolve()
    if out == runner.ROOT or runner.ROOT in out.parents:
        parser.error("Generated output must be outside this checkout")
    out.mkdir(parents=True, exist_ok=True)
    source, build = out / "source", out / "build"
    report = out / "negative-control-result.json"
    result = {"status": "failed", "commands": [], "gameStarts": 0, "nativePluginBuilt": False}
    try:
        if source.exists() or build.exists():
            raise RuntimeError("Use a new output directory: source/ and build/ must not already exist")
        cmake = runner.find_cmake(args.cmake)
        ctest = Path(cmake).with_name("ctest.exe" if os.name == "nt" else "ctest")
        pins = runner.source_pins()
        result["sourceSha256"] = pins
        # Copy only the listed authored tests, production sources and headers.
        # No Git metadata, native SDK, external fixture or binary is needed.
        for relative in pins:
            target = source / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(runner.ROOT / relative, target)
        copied = source / "src/hand_equipment.cpp"
        before = "a.instanceProvider == b.instanceProvider && a.instance == b.instance;"
        after = "a.instanceProvider == b.instanceProvider && a.instance == b.instance && a.generation == b.generation;"
        text = copied.read_text(encoding="utf-8")
        if text.count(before) != 1:
            raise RuntimeError("The expected production SameInstance seam is missing or duplicated")
        copied.write_text(text.replace(before, after), encoding="utf-8")
        witness = "f.Request(sameItem, blocked) == HIGGS_HAND_CONFLICT"
        result["mutation"] = {"file": "src/hand_equipment.cpp", "before": before, "after": after}
        result["failureWitness"] = witness
        configure = [cmake, "-S", str(source / "test"), "-B", str(build)]
        if args.generator:
            configure += ["-G", args.generator]
        generator = args.generator or os.environ.get("CMAKE_GENERATOR", "")
        if os.name == "nt" and (not generator or generator.startswith("Visual Studio")):
            configure += ["-A", "x64"]
        else:
            configure += ["-DCMAKE_BUILD_TYPE=Release"]
        commands = [
            ("configure", configure, 0),
            ("build", [cmake, "--build", str(build), "--config", "Release",
                       "--target", "equipment_claims", "--parallel", "4"], 0),
            ("ctest", [str(ctest), "--test-dir", str(build), "-C", "Release", "-R",
                       "^equipment_claims$", "--verbose", "--output-on-failure", "--no-tests=error"], 8),
        ]
        for name, command, expected in commands:
            print("+ " + subprocess.list2cmdline(command), flush=True)
            completed = subprocess.run(command, cwd=out, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, timeout=120)
            log = out / (name + ".log"); log.write_bytes(completed.stdout)
            output = completed.stdout.decode("utf-8", errors="replace")
            print(output, end="", flush=True)
            result["commands"].append({"name": name, "argv": command, "returnCode": completed.returncode,
                                       "expectedReturnCode": expected, "log": str(log)})
            if completed.returncode != expected:
                raise RuntimeError(name + " did not return the required code " + str(expected))
            if name == "ctest" and witness not in output:
                raise RuntimeError("CTest failed without the required unresolved-instance assertion")
        if runner.source_pins() != pins:
            raise RuntimeError("Original checkout changed during the negative control")
        result["status"] = "passed"
        result["meaning"] = "The compiled mutated source failed at the intended contract assertion"
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        result["error"] = str(error)
        print("ERROR: " + str(error), file=sys.stderr)
    finally:
        report.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print("Report: " + str(report), flush=True)
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
