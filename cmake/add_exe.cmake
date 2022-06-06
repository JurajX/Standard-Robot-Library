function(add_exe)
    set(options CXX_STANDARD_REQUIRED CXX_EXTENSIONS_OFF TEST)
    set(oneValueArgs NAME CXX_STANDARD)
    set(multiValueArgs SRCS LINK_LIBRARIES OPTIM_OPTIONS)
    cmake_parse_arguments(EXE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_executable(
        ${EXE_NAME}
        ${EXE_NAME}.cpp
        ${EXE_SRCS}
    )

    if(DEFINED EXE_OPTIM_OPTIONS)
        add_compile_options(
            TARGET ${EXE_NAME}
            OPTIM_OPTIONS ${EXE_OPTIM_OPTIONS}
        )
    endif(DEFINED EXE_OPTIM_OPTIONS)

    if(DEFINED EXE_LINK_LIBRARIES)
        target_link_libraries(
            ${EXE_NAME}
            PUBLIC ${EXE_LINK_LIBRARIES}
        )
    endif(DEFINED EXE_LINK_LIBRARIES)

    if(DEFINED EXE_CXX_STANDARD)
        set_target_properties(
            ${EXE_NAME} PROPERTIES
            CXX_STANDARD ${EXE_CXX_STANDARD}
            CXX_STANDARD_REQUIRED ${EXE_CXX_STANDARD_REQUIRED}
            CXX_EXTENSIONS $<NOT:${EXE_CXX_EXTENSIONS_OFF}>
        )
    endif(DEFINED EXE_CXX_STANDARD)

    if(EXE_TEST)
        target_link_libraries(
            ${EXE_NAME}
            PRIVATE gtest
            PRIVATE gmock
            PRIVATE gtest_main
        )
        add_test(
            NAME ${EXE_NAME}
            COMMAND ${EXE_NAME} --gtest_output=xml:${CMAKE_BINARY_DIR}/Testing/${EXE_NAME}_report.xml
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
    endif(EXE_TEST)
endfunction(add_exe)
