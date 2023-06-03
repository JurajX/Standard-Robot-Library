set(
    HDRS
    ${CMAKE_SOURCE_DIR}/srl/utils/data_gen.hpp
    ${CMAKE_SOURCE_DIR}/srl/utils/data_gen-impl.hpp
    ${CMAKE_SOURCE_DIR}/srl/utils/timers.hpp
    ${CMAKE_SOURCE_DIR}/srl/utils/timers-impl.hpp
)

add_library(
    srl_utils INTERFACE
    ${HDRS}
)

set_target_properties(
    srl_utils PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_link_libraries(
    srl_utils
    INTERFACE srl_common
)

target_include_directories(
    srl_utils INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)

install(TARGETS srl_utils)
install(
    FILES ${HDRS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/srl/utils
    COMPONENT development
)
