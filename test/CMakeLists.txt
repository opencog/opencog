# ADD_CXXTEST macro from http://www.cmake.org/Wiki/CMakeMacroAddCxxTest

# Make sure testing is enabled
ENABLE_TESTING()

# Use Python interpreter
FIND_PACKAGE(PythonInterp)

# Find path to the cxxtestgen.py script (NB: this stuff should move to FindCxxtest.cmake)
# CXXTEST_BIN_DIR enviroment variable must have been defined already
FIND_PATH(CXXTEST_BIN_DIR cxxtestgen.py
  $ENV{CXXTEST_BIN_DIR}
  /usr/bin
  /usr/local/bin
  $ENV{HOME}/cxxtest/include
  ${CMAKE_CURRENT_SOURCE_DIR}/cxxtest/include
  DOC "Where is cxxtest located?"
)

SET(CXXTESTGEN "${CXXTEST_BIN_DIR}/cxxtestgen.py")

MACRO(ADD_CXXTEST NAME)
  IF(PYTHONINTERP_FOUND)
    ADD_CUSTOM_COMMAND(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp
      COMMAND
        ${PYTHON_EXECUTABLE} ${CXXTESTGEN}
        --runner=ErrorPrinter
        -o ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cxxtest
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cxxtest ${ARGN}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  ENDIF(PYTHONINTERP_FOUND)

  ADD_EXECUTABLE(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp ${ARGN})
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

  ADD_TEST(${NAME} ${NAME})
ENDMACRO(ADD_CXXTEST)

#The above macro generates a single source file for all input test headers. 
#If by some reason you prefer separate compilation of each part, you may use the variation:

MACRO(ADD_CXXTEST_SEP NAME)
  IF(PYTHONINTERP_FOUND)
    # generate the parts
    FOREACH(_PART ${ARGN})
      GET_FILENAME_COMPONENT(_NAME ${_PART} NAME)
      GET_FILENAME_COMPONENT(_NAME_WE ${_PART} NAME_WE)
      ADD_CUSTOM_COMMAND(
        OUTPUT ${_NAME_WE}.cpp
        COMMAND
          ${PYTHON_EXECUTABLE} ${CXXTESTGEN}
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
        ${PYTHON_EXECUTABLE} ${CXXTESTGEN}
        --runner=ErrorPrinter --root
        -o ${NAME}_runner.cpp
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
  ENDIF(PYTHONINTERP_FOUND)

  # enumerate all generated files
  SET(PARTS ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}_runner.cpp)
  FOREACH(_PART ${ARGN})
    GET_FILENAME_COMPONENT(_PART_WE ${_PART} NAME_WE)
    SET(PARTS ${PARTS} ${_PART_WE}.cpp)
  ENDFOREACH(_PART)

  ADD_EXECUTABLE(${NAME} ${PARTS})

  ADD_TEST(${NAME} ${NAME})
ENDMACRO(ADD_CXXTEST_SEP)

add_subdirectory (AtomSpace)
add_subdirectory (NMXml)
add_subdirectory (Util)
add_subdirectory (CogServer)
