set(
    HDRS
    ${CMAKE_SOURCE_DIR}/srl/common/common.hpp
    ${CMAKE_SOURCE_DIR}/srl/common/concepts.hpp
    ${CMAKE_SOURCE_DIR}/srl/common/constants.hpp
    ${CMAKE_SOURCE_DIR}/srl/common/types.hpp
)

add_library(
    srl_common INTERFACE
    ${HDRS}
)

set_target_properties(
    srl_common PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_link_libraries(
    srl_common
    INTERFACE Eigen3::Eigen fmt::fmt
)

target_include_directories(
    srl_common INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)

install(TARGETS srl_common)
install(
    FILES ${HDRS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/srl/common
    COMPONENT development
)
