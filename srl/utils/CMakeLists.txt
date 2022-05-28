set(
    HDRS
    ${CMAKE_SOURCE_DIR}/srl/concepts.hpp
    ${CMAKE_SOURCE_DIR}/srl/constants.hpp
    ${CMAKE_SOURCE_DIR}/srl/types.hpp
    ${CMAKE_SOURCE_DIR}/srl/utils/data_gen.hpp
    ${CMAKE_SOURCE_DIR}/srl/utils/timers.hpp
)

install(
    FILES ${HDRS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT development
)

add_library(
    utils SHARED
    ${CMAKE_SOURCE_DIR}/srl/utils/data_gen-impl.hpp
    ${CMAKE_SOURCE_DIR}/srl/utils/timers-impl.hpp
    ${CMAKE_SOURCE_DIR}/srl/utils/utils.cpp
    ${HDRS}
)
add_compile_options(
    TARGET utils
    OPTIM_OPTIONS "-O3"
)

set_target_properties(
    utils PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

find_package(Eigen3 REQUIRED)
target_link_libraries(
    utils
    PUBLIC Eigen3::Eigen fmt::fmt
)

target_include_directories(
    utils PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)
