# ADD_CXXTEST macro from http://www.cmake.org/Wiki/CMakeMacroAddCxxTest

FIND_PACKAGE(Cxxtest REQUIRED)

MACRO(ADD_CXXTEST NAME)
    IF (CXXTEST_FOUND)
        ADD_CUSTOM_COMMAND(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp
            COMMAND
                "${PYTHON_EXECUTABLE}" ${CXXTEST_GEN}
                --runner=ErrorPrinter
                -o ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cxxtest
            DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cxxtest ${ARGN}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        )
    ENDIF (CXXTEST_FOUND)

    ADD_EXECUTABLE(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp ${ARGN})
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CXXTEST_INCLUDE_DIRS})

    ADD_TEST(${NAME} ${NAME})
ENDMACRO(ADD_CXXTEST)

#The above macro generates a single source file for all input test headers. 
#If by some reason you prefer separate compilation of each part, you may use the variation:

MACRO(ADD_CXXTEST_SEP NAME)
    IF(CXXTEST_FOUND)
        # generate the parts
        FOREACH(_PART ${ARGN})
            GET_FILENAME_COMPONENT(_NAME ${_PART} NAME)
            GET_FILENAME_COMPONENT(_NAME_WE ${_PART} NAME_WE)
            ADD_CUSTOM_COMMAND(
                OUTPUT ${_NAME_WE}.cpp
                COMMAND
                    ${PYTHON_EXECUTABLE} ${CXXTEST_GEN}
                    --part
                    -o ${_NAME_WE}.cpp
                    ${_NAME}
                DEPENDS ${_PART}
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            )
        ENDFOREACH(_PART)

        # generate the runner
        ADD_CUSTOM_COMMAND(
            OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}_runner.cpp
            COMMAND
                ${PYTHON_EXECUTABLE} ${CXXTEST_GEN}
                --runner=ErrorPrinter --root
                -o ${NAME}_runner.cpp
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
    ENDIF(CXXTEST_FOUND)

    # enumerate all generated files
    SET(PARTS ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}_runner.cpp)
    FOREACH(_PART ${ARGN})
        GET_FILENAME_COMPONENT(_PART_WE ${_PART} NAME_WE)
        SET(PARTS ${PARTS} ${_PART_WE}.cpp)
    ENDFOREACH(_PART)

    ADD_EXECUTABLE(${NAME} ${PARTS})

    ADD_TEST(${NAME} ${NAME})
ENDMACRO(ADD_CXXTEST_SEP)
