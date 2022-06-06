.Phony: coverage

coverage: export BUILD_TYPE=Coverage
coverage:
	${MAKE} build
	${MAKE} coverage/clean
	${MAKE} ${coverage_process}
	genhtml ${cov_dir}/coverage.info --output-directory=${cov_dir}/html


# -------------------------------------------
.Phony: coverage/build coverage/clean coverage/gcc coverage/clang

coverage/clean:
	rm -rf ${cov_dir}
	mkdir -p ${cov_dir}

coverage_process = $(if ${not_gcc},coverage/clang,coverage/gcc)

coverage/gcc:
	lcov\
		--capture --initial --directory ${project_path}\
		--output-file=${cov_dir}/cov-init.info\
		--gcov-tool ${gcov}
	${MAKE} test
	lcov\
		--capture --directory ${project_path}\
		--output-file=${cov_dir}/cov-test.info\
		--gcov-tool ${gcov}
	lcov --zerocounters --directory ${project_path}
	lcov\
		--add-tracefile ${cov_dir}/cov-init.info\
		--add-tracefile ${cov_dir}/cov-test.info\
		--output-file=${cov_dir}/cov-all.info
	lcov\
		--extract ${cov_dir}/cov-all.info\
		--output-file=${cov_dir}/coverage.info\
		${srcs}

coverage/clang: export LLVM_PROFILE_FILE=${project_path}/${cov_dir}/%m-%p.profraw
coverage/clang:
	${MAKE} test
	llvm-profdata merge\
		--output ${cov_dir}/cov.profdata\
		${cov_dir}/*.profraw
	rm ${cov_dir}/*.profraw
	llvm-cov export\
		-format=lcov\
		-instr-profile ${cov_dir}/cov.profdata\
		$(subst ${space},${space} -object ${space},${objs}) \
		${srcs} > \
		${cov_dir}/coverage.info
