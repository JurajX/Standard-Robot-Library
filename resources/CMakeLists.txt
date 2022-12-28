set(
    HDRS
    ${CMAKE_SOURCE_DIR}/resources/robot_params.hpp
)

install(
    FILES ${HDRS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT development
)

add_library(
    srl_resources INTERFACE
    ${HDRS}
)

set_target_properties(
    srl_resources PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_link_libraries(
    srl_resources
    INTERFACE srl_common
)

target_include_directories(
    srl_resources INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)
