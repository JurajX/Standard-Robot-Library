.Phony: tidy format

tidy: export COMPILER=clang
tidy: export BUILD_TYPE=tidy
tidy:
	${MAKE} build/cmake
	@echo this echo is necessary else the path is not set-up correctly &&\
	run-clang-tidy \
		-p ${project_path}/${build_dir}\
		${project_files}

format:
	clang-format -i -style=file ${project_files}
