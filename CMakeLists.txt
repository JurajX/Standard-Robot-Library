cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_COMPILER "gcc-11")
set(CMAKE_CXX_COMPILER "g++-11")

set(PROJECT_NAME "Standard Robot Library")
project(
    ${PROJECT_NAME} VERSION 0.0.0
    DESCRIPTION "A generic robot library for visualisation, planning, collision detection."
    LANGUAGES C CXX
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
include(initiate)
initiate()

add_subdirectory(srl)
add_subdirectory(examples)
add_subdirectory(tests)
add_subdirectory(resources)

if(MAKE_DOCS)
    if(NOT DEFINED MAKE_DOC_PATH)
        set(MAKE_DOC_PATH ${CMAKE_CURRENT_LIST_DIR}/build/docs)
    endif()

    add_docs(
        NAME ${PROJECT_NAME}
        FILE_DIRS srl
        EXAMPLE_PATH examples
        OUTPUT_DIRECTORY ${MAKE_DOC_PATH}
        ENABLE_PREPROCESSING MACRO_EXPANSION
    )
endif(MAKE_DOCS)
