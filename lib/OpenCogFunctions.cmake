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
# MODULE_FILE: The name of the file that defines the module. It has the same
#       name as the directory it is in, or the name of the parent directory of
#       current directory if it is in a folder named 'scm' and used to define
#       the module.

    # Check if the file exists
    IF(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${MODULE_FILE})
        MESSAGE(FATAL_ERROR "${MODULE_FILE} file does not exist in"
            ${CMAKE_CURRENT_SOURCE_DIR})
    ENDIF()

    # Clean path for specifying module paths
    # NOTE: Names of modules should be small-letters.
    # TODO: There can only be 1 folder named `scm` in path but there is no
    # check for that add it.
    STRING(REGEX MATCH
        "^(${CMAKE_HOME_DIRECTORY})/([a-z0-9/-]+)/(scm?)([a-z0-9/-]*)" ""
        ${CMAKE_CURRENT_SOURCE_DIR})

    IF(CMAKE_MATCH_0)
        SET(CLEAN_PATH "${CMAKE_HOME_DIRECTORY}/${CMAKE_MATCH_2}${CMAKE_MATCH_4}")
    ELSE()
        SET(CLEAN_PATH ${CMAKE_CURRENT_SOURCE_DIR})
    ENDIF()

    # Specify the module paths.
    STRING(REGEX MATCH
        "^(${CMAKE_HOME_DIRECTORY})/([a-z0-9/-]+)+/([a-z0-9-]+)" ""
        ${CLEAN_PATH})

    # MODULE_NAME: it is equal to the current directory name, or the current
    #       directory's parent-directory name if the current directory is scm.
    # MODULE_FILE_DIR_PATH: the directory path where the MODULE_FILE is
    #       installed.
    # MODULE_FILES_DIR_PATH: the directory path where the files associated
    #       with the module are installed/symlinked at, with the exception of
    #       the MODULE_FILE.
    SET(MODULE_NAME ${CMAKE_MATCH_3})
    SET(MODULE_FILE_DIR_PATH ${CMAKE_MATCH_2})
    SET(MODULE_FILES_DIR_PATH ${CMAKE_MATCH_2}/${CMAKE_MATCH_3})

    # Check if the file has the same name as the module_name
    IF ("${MODULE_NAME}.scm" STREQUAL "${MODULE_FILE}")
        MESSAGE("they are equal time to symlink------------")
    ELSE()
        MESSAGE(WARNING "they are not equal symlinking anyways")
    ENDIF()

ENDFUNCTION(ADD_GUILE_MODULE)
