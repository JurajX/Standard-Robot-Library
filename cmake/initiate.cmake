macro(initiate)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    include(CTest)
    include(doxygen)
    include(FetchContent)
    include(fmt)                    # needs FetchContent
    include(gtest)                  # needs FetchContent
    include(eigen)                  # needs FetchContent
    include(add_compile_options)
    include(add_exe)                # needs add_compile_options and gtest
endmacro(initiate)