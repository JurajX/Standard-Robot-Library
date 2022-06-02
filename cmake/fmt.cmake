FetchContent_Declare(
    fmt
    GIT_REPOSITORY      https://github.com/fmtlib/fmt.git
    GIT_TAG             origin/master
    GIT_PROGRESS        ON
    UPDATE_DISCONNECTED ON
)
set(BUILD_SHARED_LIBS_OLD ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCED)
FetchContent_MakeAvailable(fmt)
set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_OLD} CACHE BOOL "" FORCED)
