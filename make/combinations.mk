.Phony: combinations release debug cov

combinations: clang/rel clang/deb clang/cov gcc/rel gcc/deb gcc/cov
release: clang/rel gcc/rel
debug: clang/deb gcc/deb
cov: clang/cov gcc/cov

# -------------------------------------------
.Phony: clang/rel clang/deb clang/cov gcc/rel gcc/deb gcc/cov

clang/rel: export BUILD_TYPE=Release
clang/rel: export COMPILER=clang
clang/rel:
	${MAKE} build
	${MAKE} test

clang/deb: export BUILD_TYPE=Debug
clang/deb: export COMPILER=clang
clang/deb:
	${MAKE} build
	${MAKE} test

clang/cov: export COMPILER=clang
clang/cov:
	${MAKE} coverage

gcc/rel: export BUILD_TYPE=Release
gcc/rel: export COMPILER=gcc
gcc/rel:
	${MAKE} build
	${MAKE} test

gcc/deb: export BUILD_TYPE=Debug
gcc/deb: export COMPILER=gcc
gcc/deb:
	${MAKE} build
	${MAKE} test

gcc/cov: export COMPILER=gcc
gcc/cov:
	${MAKE} coverage
