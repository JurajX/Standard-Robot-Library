add_exe(
    NAME model_benchmark
    LINK_LIBRARIES model utils resources fmt::fmt
    OPTIM_OPTIONS "-O3"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)

add_exe(
    NAME example_model
    LINK_LIBRARIES model resources fmt::fmt
    OPTIM_OPTIONS "-O3"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)
