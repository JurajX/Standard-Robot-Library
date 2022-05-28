function(add_compile_options)
    set(options "")
    set(oneValueArgs TARGET)
    set(multiValueArgs OPTIM_OPTIONS)
    cmake_parse_arguments(COMPILE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT COMPILE_OPTIM_OPTIONS)
        list(APPEND COMPILE_OPTIM_OPTIONS "-O0")
    endif(NOT COMPILE_OPTIM_OPTIONS)

    if(
        "-O1" IN_LIST COMPILE_OPTIM_OPTIONS OR
        "-O2" IN_LIST COMPILE_OPTIM_OPTIONS OR
        "-O3" IN_LIST COMPILE_OPTIM_OPTIONS
    )
        list(APPEND COMPILE_OPTIM_OPTIONS "-D_FORTIFY_SOURCE=2")      # run-time buffer overflow detection (needs at least -O1)
    endif()

    list(APPEND COMPILE_OPTIM_OPTIONS
        -fcf-protection                     # control flow integrity protection
        -fdata-sections                     #
        -fexceptions                        # recommended for multi-threaded C code, also in combination with C++ code
        # -ffast-math                         # break strict IEEE compliance, disables setting errno,
                                            #  ...all math is finite, allows reciprocal approximations, disables signed zero
        -ffunction-sections                 #
        -fopenmp                            # links to OpenMP library, mainly for Eigen
        -fstack-clash-protection            # increased reliability of stack overflow detection
        -fstack-protector-strong            # more performant stack protector
        -fvisibility-inlines-hidden         #
        -fvisibility=default                # symbols in libraries must be explicitly exported to avoid conflicts

        -fno-asynchronous-unwind-tables     #
        -fno-default-inline                 # no automatic inlining of in-class defined member functions
        -fno-elide-constructors             #
        -fno-ident                          #
        -fno-implicit-inline-templates      #
        -fno-math-errno                     # disables setting errno
        -fno-stack-protector                #
        -fno-strict-aliasing                # fewer compiler assumptions about pointer types
        -fno-unwind-tables                  #

        -march=native                       # optimise for local architecture

        -pipe                               # avoid writing temporary files

        -Wall                               # all warnings
        -Wcast-align                        # warn for potential performance problem casts
        -Wcast-qual                         #
        -Wconversion                        # warn on type conversions that may lose data
        -Wdouble-promotion                  # warn if float is implicitely promoted to double
        -Wduplicated-cond                   # warn if if-else chain has duplicated conditions
        -Wduplicated-branches               # warn if if-else branches have duplicated code (possible problems with Eigen)
        -Wextra                             # more warnings
        -Wfloat-equal                       #
        -Wformat-nonliteral                 #
        -Wformat-security                   #
        -Wformat=2                          # warn on security issues around functions that format output (ie printf)
        -Winit-self                         #
        -Wlogical-op                        # warn about logical operations being used where bitwise were probably wanted
        -Wmisleading-indentation            # warn if indentation implies blocks where blocks do not exist
        -Wmissing-include-dirs              #
        -Wmultichar                         #
        -Wno-overlength-strings             # may happen in the nodeset compiler when complex values are directly encoded
        -Wno-unused-parameter               #
        -Wnon-virtual-dtor                  # warn if a class with virtual functions has a non-virtual destructor
        -Wnull-dereference                  # warn if a null dereference is detected
        -Wold-style-cast                    # warn for c-style casts
        -Woverloaded-virtual                # warn if you overload (not override) a virtual function
        -Wpedantic                          # warn if non-standard c++ is used
        -Wredundant-decls                   #
        -Wshadow                            # warn the user if a variable declaration shadows one from a parent context
        -Wsign-conversion                   # warn on sign conversions
        -Wstrict-overflow                   #
        -Wswitch                            # warn on missing switch case
        -Wtautological-compare              # warn if comparison always yields true/false or a tautological result
        -Wundef                             #
        -Wuninitialized                     #
        -Wunused                            # warn on anything being unused
        -Wuseless-cast                      # warn if you perform a cast to the same type
    )

    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        list(APPEND COMPILE_OPTIM_OPTIONS
            -fsanitize=undefined
            -fsanitize=address,leak
        )
    endif(CMAKE_BUILD_TYPE STREQUAL "Debug")

    target_compile_options(${COMPILE_TARGET} PRIVATE ${COMPILE_OPTIM_OPTIONS})
    target_link_options(${COMPILE_TARGET} PRIVATE ${COMPILE_OPTIM_OPTIONS})
endfunction(add_compile_options)
