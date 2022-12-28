find_package(Doxygen COMPONENTS dot)

function(add_docs)
    set(options ENABLE_PREPROCESSING MACRO_EXPANSION)
    set(oneValueArgs NAME)
    set(multiValueArgs FILE_DIRS EXAMPLE_PATH)
    cmake_parse_arguments(DOCS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(DOXYGEN_NUM_PROC_THREADS 0)
    set(DOXYGEN_EXTRACT_ALL YES)
    set(DOXYGEN_CALL_GRAPH YES)
    set(DOXYGEN_CALLER_GRAPH YES)
    set(DOXYGEN_HTML_DYNAMIC_SECTIONS YES)
    set(DOXYGEN_INTERACTIVE_SVG YES)
    set(DOXYGEN_DOT_TRANSPARENT YES)
    set(DOXYGEN_DOT_IMAGE_FORMAT svg)
    # set(DOXYGEN_UML_LOOK YES)           # uncomment to have uml style inheritance and collaboration diagrams
    set(DOXYGEN_DISABLE_INDEX YES)
    set(DOXYGEN_GENERATE_TREEVIEW YES)
    set(DOXYGEN_FULL_SIDEBAR NO)
    set(DOXYGEN_ENABLE_PREPROCESSING ${DOCS_ENABLE_PREPROCESSING})
    set(DOXYGEN_MACRO_EXPANSION  ${DOCS_MACRO_EXPANSION})
    set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/docs)
    set(DOXYGEN_EXAMPLE_PATH ${DOCS_EXAMPLE_PATH})

    set(DOXYGEN_GENERATE_HTML YES)
    set(DOXYGEN_USE_MATHJAX YES)
    set(DOXYGEN_MATHJAX_EXTENSIONS TeX/AMSmath TeX/AMSsymbols)
    set(DOXYGEN_EXTRA_PACKAGES amsmath)

    if (Doxygen_FOUND)
        doxygen_add_docs(
            documentation
            ${DOCS_FILE_DIRS}
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            COMMENT "API documentation for ${DOCS_NAME}."
        )
    endif (Doxygen_FOUND)

endfunction(add_docs)
