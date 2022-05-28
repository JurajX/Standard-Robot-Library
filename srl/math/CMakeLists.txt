set(
    HDRS
    ${CMAKE_SOURCE_DIR}/srl/concepts.hpp
    ${CMAKE_SOURCE_DIR}/srl/constants.hpp
    ${CMAKE_SOURCE_DIR}/srl/types.hpp
    ${CMAKE_SOURCE_DIR}/srl/math/math.hpp
)

install(
    FILES ${HDRS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT development
)

add_library(
    math SHARED
    ${CMAKE_SOURCE_DIR}/srl/math/math.cpp
    ${CMAKE_SOURCE_DIR}/srl/math/math-impl.hpp
    ${HDRS}
)
add_compile_options(
    TARGET math
    OPTIM_OPTIONS "-O3"
)

set_target_properties(
    math PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

find_package(Eigen3 REQUIRED)
target_link_libraries(
    math
    PUBLIC Eigen3::Eigen fmt::fmt
)

target_include_directories(
    math PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)
