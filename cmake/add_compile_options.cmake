function(add_compile_options)
    set(options "")
    set(oneValueArgs TARGET)
    set(multiValueArgs COMPILE_OPTIONS LINK_OPTIONS)
    cmake_parse_arguments(USER "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        set(CLANG true)
    elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        set(GCC true)
    else()
        message(FATAL_ERROR "Supported compilers are Clang and GCC. Got ${CMAKE_CXX_COMPILER_ID}.")
    endif()


# =========================== Warnings
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
            -Wsign-conversion                   # implicit sign conversions
            -Warith-conversion                  # implicit arithmetic conversions
        )
    endif()


# =========================== Default Options
    list(APPEND DEFAULT_OPTIONS
        -fexceptions                            # throw from c++ through c back to c++ survives
        # -fcf-protection                         # control flow integrity protection
        -fvisibility-inlines-hidden             # forbids to compare pointers to inline functions
        -fvisibility=default                    # symbols in libraries to be explicitly exported to avoid conflicts
        # -march=native                           # optimise for local architecture
        -pipe                                   # avoid writing temporary files
    )
    if(GCC)
        list(APPEND DEFAULT_OPTIONS
            -fstack-clash-protection            # increased reliability of stack overflow detection
        )
    endif()
    list(APPEND DEFAULT_OPTIONS $<$<NOT:$<CONFIG:Debug>>:-D_FORTIFY_SOURCE=2>)              # run-time buffer overflow detection (needs at least -O1)
    list(APPEND DEFAULT_OPTIONS $<$<NOT:$<BOOL:${EMSCRIPTEN}>>:-fstack-protector-strong>)   # more performant stack protector   not for emscripten
    list(APPEND DEFAULT_OPTIONS $<$<BOOL:${EMSCRIPTEN}>:-msimd128>)                         # turn on SIMD in emscripten


# =========================== Coverage Options
    list(APPEND COVERAGE_OPTIONS
        -fno-inline                             # no inline
        -g                                      # debugging information in the operating system's native format
    )
    if(CLANG)
        list(APPEND COVERAGE_OPTIONS
            -fprofile-instr-generate            # generate instrumented code to collect execution counts
            -fcoverage-mapping                  # generate coverage mapping to enable code coverage analysis
        )
    elseif(GCC)
        list(APPEND COVERAGE_OPTIONS
            -ftest-coverage                     # produce a notes file that can use to show program coverage
            -fprofile-arcs                      # records how many times each branch/call is executed
        )
    endif(CLANG)


# =========================== Debug Options
    list(APPEND DEBUG_OPTIONS
        # -fsanitize=thread                                   # thread sanitiser      cannot be used in combination with address and leak sanitisers
        # -fsanitize=memory                                   # memory sanitiser      not supported for mac
        # -fsanitize-memory-track-origins                     # track memory origin

        -fsanitize=address                                  # address sanitiser
        -fsanitize=pointer-subtract                         # instrument subtraction with pointer operands; needs address sanitiser
        -fsanitize=pointer-compare                          # instrument comparison operation; needs address sanitiser
        -fsanitize=leak                                     # leak sanitiser
        -fsanitize-address-use-after-return=always          # detect stack use after return problems
        -fsanitize-address-use-after-scope                  # sanitization of local variables

        -fsanitize=undefined                                # undefined sanitiser

        -fsanitize=integer                                  # checks for undefined or suspicious integer behavior
        -fsanitize=implicit-integer-arithmetic-value-change # catches implicit conversions that change the arithmetic value of the integer

        -fsanitize=float-divide-by-zero                     # floating point division by zero
        -fsanitize=implicit-conversion                      # checks for suspicious behavior of implicit conversions
        -fsanitize=local-bounds                             # out of bounds array indexing
        -fsanitize=nullability                              # passing, assigning, and returning null

        -fasynchronous-unwind-tables                        # generate unwind table that can be used for stack unwinding from asynchronous events
        -fno-omit-frame-pointer                             # for nicer stack traces in error messages
        -fno-optimize-sibling-calls                         # disable tail call elimination (even nicer stack traces)

        -g                                                  # debugging information in the operating system's native format
        -ggdb3                                              # debugging
    )


# =========================== Release Options
    list(APPEND RELEASE_OPTIONS
        -fno-asynchronous-unwind-tables         # see -fasynchronous-unwind-tables
        -fno-elide-constructors                 # prevents compiler from eliding the constructors
        -fno-math-errno                         # disables setting errno
        -fno-stack-protector                    #
        -fno-strict-aliasing                    # fewer compiler assumptions about pointer types
        -fno-unwind-tables                      #
    )


# =========================== Apply Options
    list(APPEND COMMON_FLAGS ${WARNINGS} ${DEFAULT_OPTIONS})
    list(APPEND USER_LINK_OPTIONS $<$<BOOL:${EMSCRIPTEN}>:-sINITIAL_MEMORY=$<IF:$<CONFIG:Debug>,512,128>MB>)
    set(USER_COMPILE_OPTIONS $<$<OR:$<CONFIG:Debug>,$<CONFIG:Coverage>>:$<FILTER:-O0 ${USER_COMPILE_OPTIONS},EXCLUDE,-O[1-9]>>)

    target_compile_options(${USER_TARGET} PRIVATE $<$<CONFIG:Coverage>:${COMMON_FLAGS};${COVERAGE_OPTIONS};${USER_COMPILE_OPTIONS}>)
    target_compile_options(${USER_TARGET} PRIVATE $<$<CONFIG:Debug>:${COMMON_FLAGS};${DEBUG_OPTIONS};${USER_COMPILE_OPTIONS}>)
    target_compile_options(${USER_TARGET} PRIVATE $<$<CONFIG:Release>:${COMMON_FLAGS};${RELEASE_OPTIONS};${USER_COMPILE_OPTIONS}>)

    target_link_options(${USER_TARGET} PRIVATE $<$<CONFIG:Coverage>:${COMMON_FLAGS};${COVERAGE_OPTIONS};${USER_LINK_OPTIONS}>)
    target_link_options(${USER_TARGET} PRIVATE $<$<CONFIG:Debug>:${COMMON_FLAGS};${DEBUG_OPTIONS};${USER_LINK_OPTIONS}>)
    target_link_options(${USER_TARGET} PRIVATE $<$<CONFIG:Release>:${COMMON_FLAGS};${RELEASE_OPTIONS};${USER_LINK_OPTIONS}>)

endfunction(add_compile_options)
