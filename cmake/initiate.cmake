macro(initiate)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    if (NOT EMSCRIPTEN)
        set(BUILD_SHARED_LIBS TRUE)
    endif()

    include(CTest)
    include(doxygen)
    include(FetchContent)
    set(FETCHCONTENT_QUIET OFF)
    include(eigen)                  # needs FetchContent
    include(fmt)                    # needs FetchContent
    include(gtest)                  # needs FetchContent
    include(add_compile_options)
    include(add_exe)                # needs add_compile_options and gtest

    if(APPLE)
        set(CMAKE_INSTALL_RPATH "@loader_path/../lib;@loader_path")
    elseif(UNIX)
        set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/")
    endif()

endmacro(initiate)
