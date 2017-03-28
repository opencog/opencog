
FIND_PROGRAM(STACK_EXECUTABLE stack)

SET(STACK_FOUND FALSE)
IF (STACK_EXECUTABLE)
    SET(MESSAGE_STATUS "You should run \"stack setup\" first. Haskell bindings will not be built.")
    EXECUTE_PROCESS(COMMAND stack ghc -- --version
                    OUTPUT_VARIABLE _GHCVERNO
                    ERROR_QUIET)
    STRING(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" GHC_VERSION "${_GHCVERNO}")

    IF ("${GHC_VERSION}" VERSION_EQUAL "8.0.2")
        SET(STACK_FOUND TRUE)
        SET(MESSAGE_STATUS "Stack found.")
    ENDIF ()
ELSE (STACK_EXECUTABLE)
    SET(MESSAGE_STATUS "Stack was not found. Haskell codes will not be built.")
ENDIF (STACK_EXECUTABLE)
MESSAGE(STATUS ${MESSAGE_STATUS})
