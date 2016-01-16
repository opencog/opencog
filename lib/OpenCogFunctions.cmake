# Copyright (C) 2016 OpenCog Foundation
#
# 1. The name of the directory and the name is the name of the module
# 2. There must be a .scm file in the same directory as the module. This file
#    on installation will be copied one directory above its current directory.
#
# Types of scm module directory structures
# 1. Modules that doen't have any c++ code modules can exist in their separate
#    directory. Examples,
#    * `/repo-name/opencog/some-module/some-module.scm` is imported by
#      `(use-modules (opencog some-module ))` if `some-module.scm` has an
#      expression `(define-module (opencog some-module))`
#    * `/repo-name/opencog/some-module/sub-module/sub-module.scm` is imported
#      by `(use-modules (opencog some-module sub-module))` if `sub-module.scm`
#      has an expression `(define-module (opencog some-module sub-module))`
# 2. If there is a c++ code then the scheme module should be under `scm/`
#    directory. And on installation or symlinking the `scm/` directory is
#    escaped. If there are more than one `scm/` directories only the first in
#    the hierarchy is escaped. As the need arises multiple `scm/` directory
#    escpaes maybe be added in the future. Examples,
#    * `/repo-name/opencog/some-module/scm/some-module.scm` is imported by
#      `(use-modules (opencog some-module ))` if `some-module.scm` has an
#      expression `(define-module (opencog some-module))`
#    * `/repo-name/opencog/some-module/scm/sub-module/sub-module.scm` is
#      imported by `(use-modules (opencog some-module sub-module))` if
#      `sub-module.scm` has an expression `(define-module (opencog some-module
#      sub-module))`

# References:
# https://www.gnu.org/software/guile/manual/guile.html#Modules-and-the-File-System
# https://www.gnu.org/software/guile/manual/guile.html#Creating-Guile-Modules

FUNCTION(PROCESS_GUILE_PATH PREFIX_DIR_PATH CURRENT_SRC_DIR FILE_NAME)
    # NOTE: Directory paths are not allowed as module names.
    # Check if directory path was passed as a file name.
    STRING(REGEX MATCH "([a-z0-9/-]+)/([a-z0-9/-]*)" "" ${FILE_NAME})
    IF(CMAKE_MATCH_2)
        MESSAGE(FATAL_ERROR "Only files found in "
            "${CURRENT_SRC_DIR} are allowed")
    ENDIF()

    # MODULE_FILE: The name of the file that defines the module. It has
    #   the same name as the directory it is in, or the name of the parent
    #   directory of current directory if it is in a folder named 'scm'.
    #   In addition this file shoule have a define-module expression
    #   for it be importable, as per guile's specification. See reference
    #   links above.
    SET(MODULE_FILE ${FILE_NAME})

    # Check if the file exists
    IF(NOT EXISTS ${CURRENT_SRC_DIR}/${MODULE_FILE})
        MESSAGE(FATAL_ERROR "${MODULE_FILE} file does not exist in "
            ${CURRENT_SRC_DIR})
    ENDIF()

    # Clean path for specifying module paths
    # NOTE: Names of modules should be small-letters.
    # TODO: There can only be 1 folder named `scm` in path but there is no
    # check for that add it.
    STRING(REGEX MATCH
        "^(${PREFIX_DIR_PATH})/([a-z0-9/-]+)/(scm?)([a-z0-9/-]*)" ""
        ${CURRENT_SRC_DIR})

    IF(CMAKE_MATCH_0)
        SET(CLEAN_PATH "${PREFIX_DIR_PATH}/${CMAKE_MATCH_2}${CMAKE_MATCH_4}")
    ELSE()
        SET(CLEAN_PATH ${CURRENT_SRC_DIR})
    ENDIF()

    # Specify the module paths.
    STRING(REGEX MATCH
        "^(${PREFIX_DIR_PATH})([a-z0-9/-]+)*/([a-z0-9-]+)" ""
        ${CLEAN_PATH})

    # MODULE_NAME: it is equal to the current directory name, or the current
    #   directory's parent-directory name if the current directory is scm.
    # MODULE_FILE_DIR_PATH: the directory path where the MODULE_FILE is
    #   installed.
    # MODULE_FILES_DIR_PATH: the directory path where the files associated
    #   with the module are installed/symlinked at, with the exception of
    #   the MODULE_FILE.
    SET(MODULE_NAME ${CMAKE_MATCH_3})
    SET(MODULE_FILE_DIR_PATH ${CMAKE_MATCH_2})
    SET(MODULE_FILES_DIR_PATH ${CMAKE_MATCH_2}/${CMAKE_MATCH_3})
    SET(GUILE_SYMLINK_DIR "${CMAKE_BINARY_DIR}/opencog/scm")
    SET(GUILE_INSTALL_DIR "${DATADIR}/scm")

    # Create symlinks in build directory mirroring the install path structure.
    # Also configure for install.
    IF ("${MODULE_NAME}.scm" STREQUAL "${MODULE_FILE}")
        EXECUTE_PROCESS(
            COMMAND ${CMAKE_COMMAND} -E make_directory ${GUILE_SYMLINK_DIR}/${MODULE_FILE_DIR_PATH}
            COMMAND ${CMAKE_COMMAND} -E create_symlink "${CURRENT_SRC_DIR}/${MODULE_FILE}" "${GUILE_SYMLINK_DIR}/${MODULE_FILE_DIR_PATH}/${MODULE_FILE}"
        )
        INSTALL(FILES
            ${MODULE_FILE}
            DESTINATION "${GUILE_INSTALL_DIR}/${MODULE_FILE_DIR_PATH}"
        )
    ELSE()
        EXECUTE_PROCESS(
            COMMAND ${CMAKE_COMMAND} -E make_directory ${GUILE_SYMLINK_DIR}/${MODULE_FILES_DIR_PATH}
            COMMAND ${CMAKE_COMMAND} -E create_symlink "${CURRENT_SRC_DIR}/${MODULE_FILE}" "${GUILE_SYMLINK_DIR}/${MODULE_FILES_DIR_PATH}/${MODULE_FILE}"
        )
        INSTALL (FILES
            ${MODULE_FILE}
            DESTINATION "${GUILE_INSTALL_DIR}/${MODULE_FILES_DIR_PATH}"
        )
    ENDIF()
ENDFUNCTION(PROCESS_GUILE_PATH)

FUNCTION(ADD_GUILE_MODULE SCHEME_FILE)
    FOREACH(FILE_NAME ${ARGV})
        PROCESS_GUILE_PATH(${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
            ${FILE_NAME}
        )
    ENDFOREACH(FILE_NAME)
ENDFUNCTION(ADD_GUILE_MODULE)
