FetchContent_Declare(
    googletest
    GIT_REPOSITORY      https://github.com/google/googletest.git
    GIT_TAG             release-1.12.1
    GIT_PROGRESS        ON
    UPDATE_DISCONNECTED ON
    SYSTEM
)
FetchContent_MakeAvailable(googletest)
