set(
    HDRS
    ${CMAKE_SOURCE_DIR}/srl/math/math.hpp
    ${CMAKE_SOURCE_DIR}/srl/math/math-impl.hpp
)

install(
    FILES ${HDRS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT development
)

add_library(
    srl_math INTERFACE
    ${HDRS}
)

set_target_properties(
    srl_math PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_link_libraries(
    srl_math
    INTERFACE srl_common
)

target_include_directories(
    srl_math INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)
