set(
    HDRS
    ${CMAKE_SOURCE_DIR}/srl/models/model.hpp
    ${CMAKE_SOURCE_DIR}/srl/models/model-impl.hpp
)

install(
    FILES ${HDRS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT development
)

add_library(
    srl_model
    ${CMAKE_SOURCE_DIR}/srl/models/model.cpp
    ${HDRS}
)

add_compile_options(TARGET srl_model)

set_target_properties(
    srl_model PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_link_libraries(
    srl_model
    PUBLIC srl_math srl_common
)

target_include_directories(
    srl_model PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)
