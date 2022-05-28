set(
    HDRS
    ${CMAKE_SOURCE_DIR}/srl/concepts.hpp
    ${CMAKE_SOURCE_DIR}/srl/constants.hpp
    ${CMAKE_SOURCE_DIR}/srl/types.hpp
    ${CMAKE_SOURCE_DIR}/srl/models/model.hpp
)

install(
    FILES ${HDRS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT development
)

add_library(
    model SHARED
    ${CMAKE_SOURCE_DIR}/srl/models/model-impl.hpp
    ${CMAKE_SOURCE_DIR}/srl/models/model.cpp
    ${HDRS}
)
add_compile_options(
    TARGET model
    OPTIM_OPTIONS "-O3"
)

set_target_properties(
    model PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

find_package(Eigen3 REQUIRED)
target_link_libraries(
    model
    PUBLIC math Eigen3::Eigen fmt::fmt
)

target_include_directories(
    model PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)
