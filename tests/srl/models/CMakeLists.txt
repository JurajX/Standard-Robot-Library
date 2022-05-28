add_exe(
    TEST
    NAME test_model_set_get
    LINK_LIBRARIES model
    OPTIM_OPTIONS "-O3"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)

add_exe(
    TEST
    NAME test_model_kin
    LINK_LIBRARIES model
    OPTIM_OPTIONS "-O3"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)

add_exe(
    TEST
    NAME test_model_dyn
    LINK_LIBRARIES model utils
    OPTIM_OPTIONS "-O3"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)
