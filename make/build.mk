.Phony: test build docs clean cclean format

test:
	${MAKE} -C ${build_dir} test

build: build/cmake
	${MAKE} -C ${build_dir}

docs: build/cmake
	${MAKE} -C ${build_dir} documentation

# does not clean external libraries that need download
clean:
	cd ${build_dir} &&\
	ls -A | grep -vo "_deps" | xargs rm -rf

cclean:
	rm -rf ${build_dir}

format:
	clang-format -i -style=file ${project_files}


# -------------------------------------------
.Phony: build/cmake build/dir

build/cmake: build/dir
	cd ${build_dir} &&\
	cmake\
		-DCMAKE_C_COMPILER=${C_COMPILER}\
		-DCMAKE_CXX_COMPILER=${CXX_COMPILER}\
		-DCMAKE_BUILD_TYPE=${BUILD_TYPE}\
		-DMAKE_DOCS=ON\
		-DMAKE_DOC_PATH=${docs_path}\
		${project_path}

build/dir:
	mkdir -p ${build_dir}
