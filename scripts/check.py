"""Scripts for running clang tidy, clang format, and for performing test coverage."""

import argparse
import os

project_path = os.path.dirname(os.path.dirname(__file__))
os.chdir(project_path)

C_CPP_EXTS = ("c", "cc", "cxx", "cpp", "c++", "h", "hh", "hxx", "hpp", "h++")
DIRS = ["srl", "examples", "tests", "resources", "scripts"]
SRC = DIRS[0]


def collect_files(paths, extensions):
    """Gather all files with given extensions in folders and subfolders specified in paths."""
    collected_files = []
    for path in paths:
        for subdir, _, files in os.walk(path):
            for file in files:
                _, extension = os.path.splitext(file)
                extension = extension.lstrip('.')
                if (extension in extensions):
                    collected_files.append(os.path.join(subdir, file))
    return collected_files


def clang_format(llvm_bin):
    """"Run clang format on all project files."""
    paths = [os.path.join(project_path, dir) for dir in DIRS]
    project_files = collect_files(paths, C_CPP_EXTS)
    command = f"{llvm_bin}/clang-format -i -style=file {' '.join(project_files)}"
    print("\n---- Running clang-format.")
    os.system(command)


def clang_tidy(llvm_bin):
    """"Run clang tidy on the project."""
    paths = [os.path.join(project_path, dir) for dir in DIRS]
    project_files = collect_files(paths, C_CPP_EXTS)

    command = f"""
        export PATH={llvm_bin}:$PATH
        cmake --preset tidy
        run-clang-tidy -quiet \\
            -p {os.path.join(project_path, "build", "tidy")} \\
            {" ".join(project_files)}
    """
    print("\n---- Running clang-tidy.")
    os.system(command)


def coverage(llvm_bin):
    """Run test coverage and generate a html coverage report."""
    print("\n---- Running coverage.")

    obj_paths = [os.path.join(project_path, "build", "coverage", dir) for dir in DIRS]
    records_path = f"{project_path}/build/coverage/records"
    profdata_file = "cov.profdata"
    command1 = f"""
        export PATH={llvm_bin}:$PATH
        export LLVM_PROFILE_FILE={records_path}/%m-%p.profraw
        rm -rf {records_path}
        cmake --preset coverage
        cmake --build -j --preset coverage
        rm -rf {records_path}
        mkdir -p {records_path}
        ctest --preset coverage
        llvm-profdata merge \\
            --output {records_path}/{profdata_file} \\
            {records_path}/*.profraw
    """
    os.system(command1)
    object_files = collect_files(obj_paths, ["o"])
    project_files = collect_files([os.path.join(project_path, SRC)], C_CPP_EXTS)
    command2 = f"""
        export PATH={llvm_bin}:$PATH
        llvm-cov export \\
            -format=lcov \\
            -instr-profile {records_path}/{profdata_file} \\
            {" -object ".join(object_files)} \\
            {" ".join(project_files)} > {records_path}/coverage.info
        genhtml {records_path}/coverage.info --output-directory={records_path}/html
    """
    os.system(command2)


def main():
    """Select function based on the given comand line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--target",
        help="Desired check function to be run, default is 'all'.",
        default="all",
        choices=["all", "coverage", "format", "tidy"],
        type=str
    )
    parser.add_argument(
        "--llvm_bin",
        required=False,
        help="Path to the llvm binary directory.",
        default="/usr/local/opt/llvm/bin",
        # default="/opt/homebrew/opt/llvm/bin",
        type=str
    )

    args = parser.parse_args()
    if not os.path.exists(args.llvm_bin):
        raise ValueError(f"The given path, {args.llvm_bin}, does not exits.")

    if args.target == "all":
        clang_format(args.llvm_bin)
        clang_tidy(args.llvm_bin)
        coverage(args.llvm_bin)
    elif args.target == "format":
        clang_format(args.llvm_bin)
    elif args.target == "tidy":
        clang_tidy(args.llvm_bin)
    elif args.target == "coverage":
        coverage(args.llvm_bin)


if __name__ == "__main__":
    main()
