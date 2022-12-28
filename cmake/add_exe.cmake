function(add_exe)
    set(options CXX_STANDARD_REQUIRED CXX_EXTENSIONS_OFF TEST)
    set(oneValueArgs NAME CXX_STANDARD)
    set(multiValueArgs SRCS LINK_LIBRARIES COMPILE_OPTIONS LINK_OPTIONS)
    cmake_parse_arguments(EXE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_executable(${EXE_NAME} ${EXE_NAME}.cpp ${EXE_SRCS})
    add_compile_options(TARGET ${EXE_NAME} COMPILE_OPTIONS ${EXE_COMPILE_OPTIONS} LINK_OPTIONS ${EXE_LINK_OPTIONS})
    target_link_libraries(${EXE_NAME} PUBLIC ${EXE_LINK_LIBRARIES})

    # if (EMSCRIPTEN)
    #     set_target_properties(${EXE_NAME} PROPERTIES SUFFIX ".html")
    #     target_link_options(${EXE_NAME} PRIVATE -sSINGLE_FILE=1)
    # endif()

    if(DEFINED EXE_CXX_STANDARD)
        set_target_properties(
            ${EXE_NAME} PROPERTIES
            CXX_STANDARD ${EXE_CXX_STANDARD}
            CXX_STANDARD_REQUIRED ${EXE_CXX_STANDARD_REQUIRED}
            CXX_EXTENSIONS $<NOT:${EXE_CXX_EXTENSIONS_OFF}>
        )
    endif(DEFINED EXE_CXX_STANDARD)

    if(EXE_TEST)
        target_link_libraries(${EXE_NAME} PRIVATE gtest gmock gtest_main)
        add_test(
            NAME ${EXE_NAME}
            COMMAND ${EXE_NAME} --gtest_output=xml:${CMAKE_BINARY_DIR}/Testing/${EXE_NAME}_report.xml
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
    endif(EXE_TEST)
endfunction(add_exe)
