FetchContent_Declare(
    tiny_xml2
    GIT_REPOSITORY      https://github.com/leethomason/tinyxml2.git
    GIT_TAG             origin/master
    GIT_PROGRESS        ON
    UPDATE_DISCONNECTED ON
    SYSTEM
)
FetchContent_MakeAvailable(tiny_xml2)
