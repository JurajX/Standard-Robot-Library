set(
    HDRS
    ${CMAKE_SOURCE_DIR}/srl/math/math.hpp
    ${CMAKE_SOURCE_DIR}/srl/math/math-impl.hpp
)

add_library(
    srl_math
    ${CMAKE_SOURCE_DIR}/srl/math/math.cpp
    ${HDRS}
)

add_compile_options(TARGET srl_math)

set_target_properties(
    srl_math PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_link_libraries(
    srl_math
    PUBLIC srl_common
)

target_include_directories(
    srl_math PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)

install(TARGETS srl_math)
install(
    FILES ${HDRS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/srl/math
    COMPONENT development
)
