add_exe(
    TEST
    NAME test_model_set_get
    LINK_LIBRARIES srl_model
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)

add_exe(
    TEST
    NAME test_model_kin
    LINK_LIBRARIES srl_model
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)

add_exe(
    TEST
    NAME test_model_dyn
    LINK_LIBRARIES srl_model srl_utils
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)
