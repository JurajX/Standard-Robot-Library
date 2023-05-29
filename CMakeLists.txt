cmake_minimum_required(VERSION 3.22)

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

add_docs(
    NAME ${PROJECT_NAME}
    FILE_DIRS srl
    EXAMPLE_PATH examples
    ENABLE_PREPROCESSING MACRO_EXPANSION
)