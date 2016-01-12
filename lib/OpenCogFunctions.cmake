# 1. The name of the directory is the name of the module
# 2. There must be a .scm file in the same directory as the module. This file
#    on installation will be copied one directory above its current directory.
# Types of scm modules
# 1. Modules that doen't have any c++ code like the nlp modules can
#    exist in their separte directory and the directory structure implies the
#    module structure.
# 2. If there is a c++ code then the scheme moduel will be under `scm/`
#    directory. And on installation or symlinking the `scm/` directory is
#    escaped.
# 3. The same logic would apply for python
# 4. Everything should be explicit

FUNCTION(ADD_GUILE_MODULE MODULE_FILE)
    # MODULE_FILE = The name of the file that defines the module
    MESSAGE("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")

    # Check if the file exists
    IF(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${MODULE_FILE})
        MESSAGE(FATAL_ERROR "${MODULE_FILE} file does not exist in"
            ${CMAKE_CURRENT_SOURCE_DIR})
    ENDIF()

    # Get current and parent cmake source directory names.
    STRING(REGEX MATCH "([a-z0-9-]+)/([^/.]+$)" "" ${CMAKE_CURRENT_SOURCE_DIR})
    SET(PARENT_SOURCE_DIR_NAME ${CMAKE_MATCH_1})
    SET(CURRENT_SOURCE_DIR_NAME ${CMAKE_MATCH_2})

    # Module name is equal to the current directory name, or the current
    # directory's parent-directory name if the current directory is scm.
    IF (${CURRENT_SOURCE_DIR_NAME} STREQUAL "scm")
        SET(MODULE_NAME ${PARENT_SOURCE_DIR_NAME})
    ELSE()
        SET(MODULE_NAME ${CURRENT_SOURCE_DIR_NAME})
    ENDIF()

    # Check if the file has the same name as the module_name
    IF ("${MODULE_NAME}.scm" STREQUAL "${MODULE_FILE}")
        MESSAGE("they are equal time to symlink------------")
    ENDIF()

    MESSAGE("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
ENDFUNCTION(ADD_GUILE_MODULE)
