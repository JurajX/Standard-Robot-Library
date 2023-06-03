FetchContent_Declare(
    fmt
    GIT_REPOSITORY      https://github.com/fmtlib/fmt.git
    GIT_TAG             9.1.0
    GIT_PROGRESS        ON
    UPDATE_DISCONNECTED ON
    SYSTEM
)
set(FMT_INSTALL ON)
FetchContent_MakeAvailable(fmt)
