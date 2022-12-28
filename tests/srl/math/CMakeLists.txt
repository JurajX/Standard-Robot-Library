add_exe(
    TEST
    NAME test_math
    LINK_LIBRARIES srl_math Eigen3::Eigen
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)
