SHELL = /bin/zsh
# project constants
comma := ,
empty :=
space := ${empty} ${empty}

src_dir := srl
docs_dir := docs
build := build
coverage := coverage

project_path := ${CURDIR}
src_path := ${project_path}/${src_dir}
docs_path := ${project_path}/${build}/${docs_dir}

extensions := (c|cc|cxx|cpp|c\+\+|h|hh|hxx|hpp|h\+\+)
exclude_dirs := (${build}|${docs_dir}|\.git)
srcs := $(shell find -E ${src_path} -type f -regex ".*\.${extensions}")
project_files := $(shell \
    find -E ${project_path}\
        -type d -regex "${project_path}/${exclude_dirs}" -prune -o\
        -type f -regex ".*\.${extensions}" -print)

# recursive variables
build_dir = ${build}/${COMPILER}/${BUILD_TYPE}
cov_dir = ${build_dir}/${coverage}
objs = $(shell find -E ${build_dir} -type f -regex ".*\.o")

# 'gcc' or 'clang'; gcc is the default
COMPILER ?= gcc
# 'Release' or 'Debug'; Debug is the default
BUILD_TYPE ?= Release

# set-up compilers
ifneq (${COMPILER}, gcc)
    ifneq (${COMPILER}, clang)
        $(error Unknown compiler '${COMPILER}'. Must be 'gcc' or 'clang')
    endif
endif
not_gcc = $(subst gcc,${empty},${COMPILER})
C_COMPILER = $(if ${not_gcc},clang,gcc-11)
CXX_COMPILER = $(if ${not_gcc},clang++,g++-11)

# needed for OS X to avoid Apple clang
gcov := /usr/local/opt/gcc@11/bin/gcov-11
llvm_path := /usr/local/opt/llvm
path := ${PATH}
export PATH = $(if ${not_gcc},${llvm_path}/bin:${path},${path})
export LDFLAGS = $(if ${not_gcc},-L${llvm_path}/lib,${empty})
export CPPFLAGS = $(if ${not_gcc},-I${llvm_path}/include,${empty})

.Phony: all cleanall

all: tidy format docs
	${MAKE} combinations

cleanall:
	rm -rf ${build}

# needs:
#  - targets: build, test, coverage
# provides:
#  - targets: combinations, release, debug, cov
include ${project_path}/make/combinations.mk

# needs:
#  - variables: C_COMPILER, CXX_COMPILER, BUILD_TYPE, project_path, build_dir, docs_path
# provides:
#  - targets:   test, build, docs, clean, cclean
include ${project_path}/make/build.mk

# needs:
#  - targets:   build, test
#  - variables: not_gcc, project_path, srcs, objs, cov_dir, space
#  - programme: gcov, lcov, llvm-profdata, llvm-cov
# provides:
#  - targets:   coverage
include ${project_path}/make/coverage.mk

# needs:
#  - targets:   build/cmake
#  - variables: project_path, build_dir
#  - programme: run-clang-tidy, clang-format
# provides:
#  - targets:   tidy, format
include ${project_path}/make/checks.mk
