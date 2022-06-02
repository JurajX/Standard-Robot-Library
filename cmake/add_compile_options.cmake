# Remove optimisation flags from the automatic Cmake compile flags as they are set manualy later.
string(REGEX REPLACE "-O." "" CMAKE_C_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})
string(REGEX REPLACE "-O." "" CMAKE_C_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})
string(REGEX REPLACE "-O." "" CMAKE_C_FLAGS_RELWITHDEBINFO ${CMAKE_C_FLAGS_RELWITHDEBINFO})
string(REGEX REPLACE "-O." "" CMAKE_C_FLAGS_MINSIZEREL ${CMAKE_C_FLAGS_MINSIZEREL})
string(REGEX REPLACE "-O." "" CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG})
string(REGEX REPLACE "-O." "" CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})
string(REGEX REPLACE "-O." "" CMAKE_CXX_FLAGS_RELWITHDEBINFO ${CMAKE_CXX_FLAGS_RELWITHDEBINFO})
string(REGEX REPLACE "-O." "" CMAKE_CXX_FLAGS_MINSIZEREL ${CMAKE_CXX_FLAGS_MINSIZEREL})

function(add_compile_options)
    set(options "")
    set(oneValueArgs TARGET)
    set(multiValueArgs OPTIM_OPTIONS)
    cmake_parse_arguments(COMPILE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        set(CLANG true)
    endif()

    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        set(GCC true)
    endif()

    if("${CMAKE_BUILD_TYPE}" STREQUAL "Coverage")
        set(COVERAGE true)
    endif()

    if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        set(DEBUG true)
    endif()

    list(APPEND WARNINGS
        -Wall                                   # all warnings
        -Wextra                                 # extra warnings
        -Wpedantic                              # non-standard c++ is used
        -Wconversion                            # type conversions that may alter value
        -Wshadow                                # a variable declaration shadows one from a parent context
        -Wundef                                 # an undefined identifier is evaluated
        -Wunused                                # anything being unused
        -Wformat=2                              # security issues around functions that format output (ie printf)
        -Wcast-align                            # a pointer cast increases required alignment
        -Wcast-qual                             # a pointer cast removes a type qualifier
        -Wdouble-promotion                      # a float is implicitely promoted to double
        -Wfloat-equal                           # floating-point values used in equality comparisons
        -Wnon-virtual-dtor                      # a class with virtual functions has a non-virtual destructor
        -Wnull-dereference                      # a null dereference is detected
        -Wold-style-cast                        # c-style casts
        -Woverloaded-virtual                    # overloaded (not overrided) virtual function
        -Wredundant-decls                       # a variable declared more than once in the same scope
        -Wno-overlength-strings                 # a string constants that are longer than the length specified in the C standard
        -Wsuggest-override                      # overriding virtual functions that are not marked override
        -Wextra-semi                            # redundant semicolons after in-class function definitions
    )
    if(CLANG)
        list(APPEND WARNINGS
            -Wmost                              # most warnings
        )
    elseif(GCC)
        list(APPEND WARNINGS
            -Wuseless-cast                      # unnecessary cast
        )
    endif()

    list(APPEND COMPILE_OPTIONS
        -fexceptions                            # throw from c++ through c back to c++ survives
        -fcf-protection                         # control flow integrity protection
        -fopenmp                                # links to OpenMP library, mainly for Eigen
        -fstack-protector-strong                # more performant stack protector
        # -fstack-clash-protection                # increased reliability of stack overflow detection
        -fvisibility-inlines-hidden             # forbids to compare pointers to inline functions
        -fvisibility=default                    # symbols in libraries to be explicitly exported to avoid conflicts
        -march=native                           # optimise for local architecture
        -pipe                                   # avoid writing temporary files
    )

    if((NOT COMPILE_OPTIM_OPTIONS) OR COVERAGE)
        list(APPEND COMPILE_OPTIM_OPTIONS "-O0")
        list(REMOVE_ITEM COMPILE_OPTIM_OPTIONS "-O1" "-O2" "-O3")
    endif()

    if((NOT DEBUG) AND ("-O1" IN_LIST COMPILE_OPTIM_OPTIONS OR "-O2" IN_LIST COMPILE_OPTIM_OPTIONS OR "-O3" IN_LIST COMPILE_OPTIM_OPTIONS))
        list(APPEND COMPILE_OPTIONS
            "-D_FORTIFY_SOURCE=2"               # run-time buffer overflow detection (needs at least -O1)
        )
    endif()

    list(APPEND COMPILE_COVERAGE_OPTIONS
        -fno-inline                             # no inline
        -g                                      # debugging information in the operating system's native format
    )
    if(CLANG)
        list(APPEND COMPILE_COVERAGE_OPTIONS
            -fprofile-instr-generate            # generate instrumented code to collect execution counts
            -fcoverage-mapping                  # generate coverage mapping to enable code coverage analysis
        )
    elseif(GCC)
        list(APPEND COMPILE_COVERAGE_OPTIONS
            -ftest-coverage                     # produce a notes file that can use to show program coverage
            -fprofile-arcs                      # records how many times each branch/call is executed
        )
    endif(CLANG)

    list(APPEND COMPILE_DEBUG_OPTIONS
        -fsanitize=address                      # address sanitiser
        -fsanitize=leak                         # leak sanitiser
        # -fsanitize=thread                       # thread sanitiser      cannot be used in combination with address and leak sanitisers
        # -fsanitize=memory                       # memory sanitiser      not supported for mac
        # -fsanitize-memory-track-origins         # track memory origin   used in combination with memory
        -fsanitize=undefined                    # undefined sanitiser

        -fasynchronous-unwind-tables            # generate unwind table that can be used for stack unwinding from asynchronous events
        -fno-omit-frame-pointer                 # for nicer stack traces in error messages
        -fno-optimize-sibling-calls             # disable tail call elimination
        -g                                      # debugging information in the operating system's native format
        # -ggdb3                                  # debugging
    )

    if(CLANG)
        list(APPEND COMPILE_DEBUG_OPTIONS
            # parts of undefined sanitiser that are not enabled by default
            -fsanitize=float-divide-by-zero                     #  - floating point division by zero
            -fsanitize=integer                                  #  - checks for undefined or suspicious integer behavior
            -fsanitize=implicit-conversion                      #  - checks for suspicious behavior of implicit conversions
            -fsanitize=local-bounds                             #  - out of bounds array indexing
            -fsanitize=nullability                              #  - passing, assigning, and returning null
            -fsanitize=implicit-integer-arithmetic-value-change #  - catches implicit conversions that change the arithmetic value of the integer
        )
    elseif(GCC)
        list(APPEND COMPILE_DEBUG_OPTIONS
            -fsanitize=pointer-compare          # instrument comparison operation; needs address sanitiser
            -fsanitize=pointer-subtract         # instrument subtraction with pointer operands; needs address sanitiser
            -fsanitize-address-use-after-scope  # sanitization of local variables
        )
    endif()

    list(APPEND COMPILE_RELEASE_OPTIONS
        -fno-asynchronous-unwind-tables         # see -fasynchronous-unwind-tables
        -fno-elide-constructors                 # prevents compiler from eliding the constructors
        -fno-math-errno                         # disables setting errno
        -fno-stack-protector                    #
        -fno-strict-aliasing                    # fewer compiler assumptions about pointer types
        -fno-unwind-tables                      #
        # -ffast-math                             # break strict IEEE compliance, disables setting errno,
                                                #  ...all math is finite, allows reciprocal approximations, disables signed zero
    )

    list(APPEND COMMON_FLAGS
        ${WARNINGS}
        ${COMPILE_OPTIONS}
        ${COMPILE_OPTIM_OPTIONS}
    )

    target_compile_options(${COMPILE_TARGET} PRIVATE "$<$<CONFIG:Coverage>:${COMMON_FLAGS};${COMPILE_COVERAGE_OPTIONS}>")
    target_compile_options(${COMPILE_TARGET} PRIVATE "$<$<CONFIG:Debug>:${COMMON_FLAGS};${COMPILE_DEBUG_OPTIONS}>")
    target_compile_options(${COMPILE_TARGET} PRIVATE "$<$<CONFIG:Release>:${COMMON_FLAGS};${COMPILE_RELEASE_OPTIONS}>")

    target_link_options(${COMPILE_TARGET} PRIVATE "$<$<CONFIG:Coverage>:${COMMON_FLAGS};${COMPILE_COVERAGE_OPTIONS}>")
    target_link_options(${COMPILE_TARGET} PRIVATE "$<$<CONFIG:Debug>:${COMMON_FLAGS};${COMPILE_DEBUG_OPTIONS}>")
    target_link_options(${COMPILE_TARGET} PRIVATE "$<$<CONFIG:Release>:${COMMON_FLAGS};${COMPILE_RELEASE_OPTIONS}>")

endfunction(add_compile_options)
