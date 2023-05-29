add_exe(
    NAME model_benchmark
    LINK_LIBRARIES srl_model srl_utils srl_resources fmt::fmt
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)

add_exe(
    NAME example_model
    LINK_LIBRARIES srl_model srl_resources fmt::fmt
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)
