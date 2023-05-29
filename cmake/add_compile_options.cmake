function(add_compile_options)
    set(options "")
    set(oneValueArgs TARGET)
    set(multiValueArgs COMPILE_OPTIONS LINK_OPTIONS)
    cmake_parse_arguments(USER "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        set(CLANG TRUE)
    elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        set(GCC TRUE)
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
        -Wold-style-cast                        # c-style casts
        -Woverloaded-virtual                    # overloaded (not overrided) virtual function
        -Wredundant-decls                       # a variable declared more than once in the same scope
        -Wno-overlength-strings                 # a string constants that are longer than the length specified in the C standard
        -Wsuggest-override                      # overriding virtual functions that are not marked override
        -Wextra-semi                            # redundant semicolons after in-class function definitions

        $<$<BOOL:${CLANG}>:-Wmost>              # most warnings
        $<$<BOOL:${CLANG}>:-Wnull-dereference>  # a null dereference is detected

        $<$<BOOL:${GCC}>:-Wuseless-cast>        # unnecessary cast
        $<$<BOOL:${GCC}>:-Wsign-conversion>     # implicit sign conversions
        $<$<BOOL:${GCC}>:-Warith-conversion>    # implicit arithmetic conversions
    )


# =========================== Default Options
    list(APPEND DEFAULT_OPTIONS
        -fexceptions                                                # throw from c++ through c back to c++ survives
        -fvisibility-inlines-hidden                                 # forbids to compare pointers to inline functions
        -fvisibility=default                                        # symbols in libraries to be explicitly exported to avoid conflicts
        -pipe                                                       # avoid writing temporary files
        -fcf-protection                                             # control flow integrity protection
        $<$<BOOL:${GCC}>:-fstack-clash-protection>                  # increased reliability of stack overflow detection
        $<$<NOT:$<BOOL:${EMSCRIPTEN}>>:-fstack-protector-strong>    # more performant stack protector   not for emscripten
        $<$<NOT:$<CONFIG:Debug>>:-D_FORTIFY_SOURCE=2>               # run-time buffer overflow detection (needs at least -O1)
        $<$<BOOL:${EMSCRIPTEN}>:-msimd128>                          # turn on SIMD in emscripten
        # -march=native                                               # optimise for local architecture
    )






# =========================== Coverage Options
    list(APPEND COVERAGE_OPTIONS
        -fno-inline                                     # no inline
        -g                                              # debugging information in the operating system's native format
        $<$<BOOL:${CLANG}>:-fprofile-instr-generate>    # generate instrumented code to collect execution counts
        $<$<BOOL:${CLANG}>:-fcoverage-mapping>          # generate coverage mapping to enable code coverage analysis
        $<$<BOOL:${GCC}>:-ftest-coverage>               # produce a notes file that can use to show program coverage
        $<$<BOOL:${GCC}>:-fprofile-arcs>                # records how many times each branch/call is executed
    )


# =========================== Debug Options
    set(GCC_FIXED TRUE)
    if (APPLE AND GCC)
        set(GCC_FIXED FALSE)
    endif()

    list(APPEND DEBUG_OPTIONS
        # -fsanitize=thread                                   # thread sanitiser      cannot be used in combination with address and leak sanitisers
        # -fsanitize=memory                                   # memory sanitiser      not supported for mac
        # -fsanitize-memory-track-origins                     # track memory origin

        $<$<BOOL:${GCC_FIXED}>:-fsanitize=address>                  # address sanitiser
        $<$<BOOL:${GCC_FIXED}>:-fsanitize=pointer-subtract>         # instrument subtraction with pointer operands; needs address sanitiser
        $<$<BOOL:${GCC_FIXED}>:-fsanitize=pointer-compare>          # instrument comparison operation; needs address sanitiser
        $<$<BOOL:${GCC_FIXED}>:-fsanitize=leak>                     # leak sanitiser; needs address sanitiser
        $<$<BOOL:${GCC_FIXED}>:-fsanitize-address-use-after-scope>  # sanitization of local variables; needs address sanitiser

        $<$<BOOL:${GCC_FIXED}>:-fsanitize=undefined>                # undefined sanitiser
        $<$<BOOL:${GCC_FIXED}>:-fsanitize=float-divide-by-zero>     # floating point division by zero

        $<$<BOOL:${CLANG}>:-fsanitize-address-use-after-return=always>          # detect stack use after return problems; needs address sanitiser
        $<$<BOOL:${CLANG}>:-fsanitize=integer>                                  # checks for undefined or suspicious integer behavior
        $<$<BOOL:${CLANG}>:-fsanitize=implicit-integer-arithmetic-value-change> # implicit conversions that change the value of the integer
        $<$<BOOL:${CLANG}>:-fsanitize=implicit-conversion>                      # checks for suspicious behavior of implicit conversions
        $<$<BOOL:${CLANG}>:-fsanitize=local-bounds>                             # out of bounds array indexing
        $<$<BOOL:${CLANG}>:-fsanitize=nullability>                              # passing, assigning, and returning null

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
