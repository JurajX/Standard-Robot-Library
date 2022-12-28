"""Scripts for building and running test for a specified preset."""

import argparse
import os
import subprocess
import platform
import re


def create_environ():
    """Get the current environment and remove emscripten's old node version."""
    is_win = platform.system() == "Windows"
    separator = ";" if is_win else ":"
    bad_node = "node\\\\\d\d\.\d\d\.(:? _|\w+)\\\\" if is_win else "node/\d\d\.\d\d\.(:? _|\w+)/"

    environ = os.environ.copy()
    paths = environ['PATH'].split(separator)
    paths = [path for path in paths if re.search(bad_node, path) is None]
    environ['PATH'] = separator.join(paths)
    return environ


def configure(preset):
    """Configure preset."""
    emcmake = "emcmake" if "ems" in preset else ""
    denv = "-DCMAKE_CROSSCOMPILING_EMULATOR=node" if "ems" in preset else ""
    command = f"{emcmake} cmake --preset {preset} {denv}"
    subprocess.run(command, shell=True, env=create_environ(), check=True)


def build(preset, num_cores=None):
    """Build preset."""
    cores = f"-j{num_cores}" if num_cores is not None else ""
    command = f"cmake --build --preset {preset} {cores}"
    subprocess.run(command, shell=True, env=create_environ(), check=True)


def test(preset):
    """Test preset."""
    command = f"ctest --output-on-failure --preset {preset}"
    subprocess.run(command, shell=True, env=create_environ(), check=True)


def docs():
    """Create documentation."""
    configure("docs")
    command = "cmake --build --preset docs --target documentation"
    os.system(command)


def list_presets():
    """List all presets in the repository."""
    command = "cmake --list-presets"
    result = subprocess.run(command, shell=True, capture_output=True, text=True, check=True)
    out = result.stdout.replace('"', '').replace(' ', '')
    all_presets = out.split('\n')
    all_presets.remove("coverage")
    all_presets.remove("tidy")
    return all_presets[2:-1]


def main():
    """Select function based on the given comand line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--preset", required=True, help="The name of the preset.", type=str)
    parser.add_argument(
        "--target",
        required=False,
        help="Desired target to run, default is 'all'.",
        default="all",
        choices=["all", "configure", "build", "test"],
        type=str
    )
    parser.add_argument("-j", required=False, help="The number of cores to use for build.", default=None, type=int)
    args = parser.parse_args()

    presets = list_presets()
    if args.preset not in presets:
        intend = '\n    '
        raise ValueError(
            f"The given preset, {args.preset}, does not exits. Use one of the following:{intend}{intend.join(presets)}"
        )

    if args.preset == "docs":
        docs()
        return

    if args.target == "all":
        configure(args.preset)
        build(args.preset, args.j)
        test(args.preset)
    elif args.target == "configure":
        configure(args.preset)
    elif args.target == "build":
        build(args.preset, args.j)
    elif args.target == "test":
        build(args.preset, args.j)
        test(args.preset)


if __name__ == "__main__":
    main()
