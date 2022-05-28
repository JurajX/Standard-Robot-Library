all: format test docs

BUILD_TYPE := Release

test: build
	$(MAKE) -C build test

build: build/cmake
	$(MAKE) -C build

docs: build/cmake
	$(MAKE) -C build documentation

build/cmake: build/dir
	cd build &&\
	cmake\
		-DCMAKE_BUILD_TYPE=${BUILD_TYPE}\
		-DMAKE_DOCS=ON\
		..

build/dir:
	mkdir -p build

cclean:
	rm -rf build docs

clean:
	cd build &&\
	ls -A | grep -vo "_deps" | xargs rm -rf

format:
	find -E . \
		-type d -regex '\./(build|docs|\.git)' -prune -o \
		-type f -regex ".*\.(c|cc|cxx|cpp|c\+\+|h|hh|hxx|hpp|h\+\+)" -print | \
	xargs clang-format -i -style=file
