find_package(Eigen3 REQUIRED)
add_exe(
    TEST
    NAME test_math
    LINK_LIBRARIES math Eigen3::Eigen
    OPTIM_OPTIONS "-O3"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)
