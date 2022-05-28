set(
    HDRS
    ${CMAKE_SOURCE_DIR}/srl/concepts.hpp
    ${CMAKE_SOURCE_DIR}/srl/constants.hpp
    ${CMAKE_SOURCE_DIR}/srl/types.hpp
    ${CMAKE_SOURCE_DIR}/resources/robot_params.hpp
)

install(
    FILES ${HDRS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT development
)

add_library(
    resources SHARED
    ${CMAKE_SOURCE_DIR}/resources/resources.cpp
    ${HDRS}
)
add_compile_options(
    TARGET resources
    OPTIM_OPTIONS "-O3"
)

set_target_properties(
    resources PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

find_package(Eigen3 REQUIRED)
target_link_libraries(
    resources
    PUBLIC Eigen3::Eigen
)

target_include_directories(
    resources PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)
