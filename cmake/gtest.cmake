include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY      https://github.com/google/googletest.git
    GIT_TAG             origin/main
    GIT_PROGRESS        ON
    UPDATE_DISCONNECTED ON
)
set(BUILD_SHARED_LIBS_OLD ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCED)
FetchContent_MakeAvailable(googletest)
set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_OLD} CACHE BOOL "" FORCED)
