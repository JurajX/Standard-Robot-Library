FetchContent_Declare(
    eigen
    GIT_REPOSITORY      https://gitlab.com/libeigen/eigen.git
    GIT_TAG             origin/master
    GIT_PROGRESS        ON
    UPDATE_DISCONNECTED ON
    SYSTEM
)
FetchContent_MakeAvailable(eigen)

set(BLA_VENDOR Generic)

find_package(BLAS)
if(BLAS_FOUND)
    target_link_libraries(eigen INTERFACE BLAS::BLAS)
    add_compile_definitions(EIGEN_USE_BLAS)
endif()

# find_package(LAPACK)
# if(LAPACK_FOUND)
#     target_link_libraries(eigen INTERFACE LAPACK::LAPACK)
#     add_compile_definitions(EIGEN_USE_LAPACKE)
# endif()
