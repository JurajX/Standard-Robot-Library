add_exe(
    TEST
    NAME test_data_gen
    LINK_LIBRARIES srl_utils
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)

add_exe(
    TEST
    NAME test_timers
    LINK_LIBRARIES srl_utils
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)
