add_exe(
    NAME example_math
    LINK_LIBRARIES math fmt::fmt
    OPTIM_OPTIONS "-O3"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED
    CXX_EXTENSIONS_OFF
)
