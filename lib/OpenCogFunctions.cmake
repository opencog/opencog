# Copyright (C) 2016 OpenCog Foundation
#
# 1. The name of the directory and the name of its parent directories is the
#    name of the module.
# 2. There must be a .scm file in the same directory as the module. This file
#    on installation will be copied one directory above its current directory.
#
# Types of scm module directory structures
# 1. Modules that don't have any c++ code modules can exist in their separate
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
#    Note that in the future the same could be done for python modules with
#    `py/` or `python/` directories.

# References:
# https://www.gnu.org/software/guile/manual/guile.html#Modules-and-the-File-System
# https://www.gnu.org/software/guile/manual/guile.html#Creating-Guile-Modules
# https://www.gnu.org/software/guile/manual/guile.html#Installing-Site-Packages

# Definitions:
#
# * MODULE_FILE: The name of the file that defines the module. It has
#   the same name as the directory it is in, or the name of the parent
#   directory of current directory if it is in a folder named 'scm'.
#   In addition this file shoule have a define-module expression
#   for it be importable, as per guile's specification. See reference
#   links above.

# ----------------------------------------------------------------------------
FUNCTION(PROCESS_GUILE_PATH PREFIX_DIR_PATH FILE_NAME)
    # NOTE: Directory paths are not allowed as module names.
    # Check if directory path was passed as a file name.
    STRING(REGEX MATCH "([a-z0-9/-]+)/([a-z0-9/-]*)" "" ${FILE_NAME})
    IF(CMAKE_MATCH_2)
        MESSAGE(FATAL_ERROR "Only files found in "
            "${CMAKE_CURRENT_SOURCE_DIR} are allowed")
    ENDIF()

    # Check if the file exists
    IF(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${FILE_NAME})
        MESSAGE(FATAL_ERROR "${FILE_NAME} file does not exist in "
            ${CMAKE_CURRENT_SOURCE_DIR})
    ENDIF()

    # Clean path for specifying module paths
    # NOTE: Names of modules should be small-letters.
    # TODO: There can only be 1 folder named `scm` in path but there is no
    # check for that, add it.
    STRING(REGEX MATCH
        "^(${PREFIX_DIR_PATH})/([a-z0-9/-]+)/(scm?)([a-z0-9/-]*)" ""
        ${CMAKE_CURRENT_SOURCE_DIR})

    IF(CMAKE_MATCH_0)
        SET(CLEAN_PATH "${PREFIX_DIR_PATH}/${CMAKE_MATCH_2}${CMAKE_MATCH_4}")
    ELSE()
        SET(CLEAN_PATH ${CMAKE_CURRENT_SOURCE_DIR})
    ENDIF()

    # Specify the module paths.
    STRING(REGEX MATCH
        "^(${PREFIX_DIR_PATH})([a-z0-9/-]+)*/([a-z0-9-]+)" ""
        ${CLEAN_PATH})

    # MODULE_NAME: it is equal to the current directory name, or the current
    #   directory's parent-directory name if the current directory is scm.
    # MODULE_FILE_DIR_PATH: the directory path where the MODULE_FILE is
    #   installed.
    # MODULE_DIR_PATH: the directory path where the files associated
    #   with the module are installed/symlinked at, with the exception of
    #   the MODULE_FILE.
    SET(MODULE_NAME ${CMAKE_MATCH_3} PARENT_SCOPE)
    SET(MODULE_FILE_DIR_PATH ${CMAKE_MATCH_2} PARENT_SCOPE)
    SET(MODULE_DIR_PATH ${CMAKE_MATCH_2}/${CMAKE_MATCH_3} PARENT_SCOPE)
ENDFUNCTION(PROCESS_GUILE_PATH)

# ----------------------------------------------------------------------------
# This configures the install and symlink paths for each file, passed to it,
# based on the value of the variables MODULE_NAME, MODULE_FILE_DIR_PATH and
# MODULE_DIR_PATH in the PARENT_SCOPE.
FUNCTION(PROCESS_MODULE_STRUCTURE FILE_NAME)
    SET(GUILE_SYMLINK_DIR "${CMAKE_BINARY_DIR}/opencog/scm")
    SET(GUILE_INSTALL_DIR "${DATADIR}/scm")

    # Create symlinks in build directory mirroring the install path structure.
    # Also configure for install.
    IF ("${MODULE_NAME}.scm" STREQUAL "${FILE_NAME}")
        EXECUTE_PROCESS(
            COMMAND ${CMAKE_COMMAND} -E make_directory ${GUILE_SYMLINK_DIR}/${MODULE_FILE_DIR_PATH}
            COMMAND ${CMAKE_COMMAND} -E create_symlink "${CMAKE_CURRENT_SOURCE_DIR}/${FILE_NAME}" "${GUILE_SYMLINK_DIR}/${MODULE_FILE_DIR_PATH}/${FILE_NAME}"
        )
        SET(FILE_INSTALL_PATH "${GUILE_INSTALL_DIR}/${MODULE_FILE_DIR_PATH}"
            PARENT_SCOPE
        )
    ELSE()
        EXECUTE_PROCESS(
            COMMAND ${CMAKE_COMMAND} -E make_directory ${GUILE_SYMLINK_DIR}/${MODULE_DIR_PATH}
            COMMAND ${CMAKE_COMMAND} -E create_symlink "${CMAKE_CURRENT_SOURCE_DIR}/${FILE_NAME}" "${GUILE_SYMLINK_DIR}/${MODULE_DIR_PATH}/${FILE_NAME}"
        )
        SET(FILE_INSTALL_PATH "${GUILE_INSTALL_DIR}/${MODULE_DIR_PATH}"
            PARENT_SCOPE
        )
    ENDIF()
ENDFUNCTION(PROCESS_MODULE_STRUCTURE)

# ----------------------------------------------------------------------------
FUNCTION(ADD_GUILE_MODULE)
    FOREACH(FILE_NAME ${ARGV})
        PROCESS_GUILE_PATH(${CMAKE_SOURCE_DIR} ${FILE_NAME})
        PROCESS_MODULE_STRUCTURE(${FILE_NAME})

        # The install configuration isn't part of PROCESS_GUILE_PATH function
        # so as to avoid "Command INSTALL() is not scriptable" error, when
        # using it in symlinking scheme files during code-generation
        # by the OPENCOG_ADD_ATOM_TYPES macro.
        INSTALL (FILES
            ${FILE_NAME}
            DESTINATION ${FILE_INSTALL_PATH}
        )
    ENDFOREACH(FILE_NAME)
ENDFUNCTION(ADD_GUILE_MODULE)

# ----------------------------------------------------------------------------
FUNCTION(ADD_GUILE_MODULE2)
    # NOTE: Change PREFIX_DIR_PATH variable if a choice is made to adapt
    # guile's site-package convention.
    SET(PREFIX_DIR_PATH "${DATADIR}/scm")
    SET(options "")  # This is used only as a place-holder
    SET(oneValueArgs MODULE_DESTINATION)
    SET(multiValueArgs FILES)
    CMAKE_PARSE_ARGUMENTS(SCM "${options}" "${oneValueArgs}"
        "${multiValueArgs}" ${ARGN})

    # NOTE:  The keyword arguments 'FILES' and 'MODULE_DESTINATION' are
    # required.
    IF((DEFINED SCM_FILES) AND (DEFINED SCM_MODULE_DESTINATION))
        FOREACH(FILE_NAME ${SCM_FILES})
            # Check if the file exists in the current source directory.
            IF(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${FILE_NAME})
                MESSAGE(FATAL_ERROR "${FILE_NAME} file does not exist in "
                ${CMAKE_CURRENT_SOURCE_DIR})
            ENDIF()

            # Specify module paths.
            STRING(REGEX MATCH
                "^(${PREFIX_DIR_PATH})([a-z0-9/-]+)*/([a-z0-9-]+)" ""
                ${SCM_MODULE_DESTINATION})

            # MODULE_NAME: it is equal to the MODULE_DESTINATION directory name
            # MODULE_FILE_DIR_PATH: the directory path where the MODULE_FILE is
            #   installed.
            # MODULE_DIR_PATH: the directory path where the files associated
            #   with the module are installed/symlinked at, with the exception
            #   of the MODULE_FILE.
            SET(MODULE_NAME ${CMAKE_MATCH_3})
            SET(MODULE_FILE_DIR_PATH ${CMAKE_MATCH_2})
            SET(MODULE_DIR_PATH ${CMAKE_MATCH_2}/${CMAKE_MATCH_3})

            PROCESS_MODULE_STRUCTURE(${FILE_NAME})
            # NOTE: The install configuration isn't part of
            # PROCESS_MODULE_STRUCTURE function so as to avoid "Command
            # INSTALL() is not scriptable" error, when using it in symlinking
            # scheme files during code-generation by the OPENCOG_ADD_ATOM_TYPES
            # macro.
            INSTALL (FILES
                ${FILE_NAME}
                DESTINATION ${FILE_INSTALL_PATH}
            )

        ENDFOREACH()
    ELSE()
        IF(NOT DEFINED SCM_FILES)
            MESSAGE(FATAL_ERROR "The keyword argument 'FILES' is not set in "
                ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE})
        ENDIF()

        IF(NOT DEFINED MODULE_DESTINATION)
            MESSAGE(FATAL_ERROR "The keyword argument 'MODULE_DESTINATION' "
            "is not set in "
            ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE})
        ENDIF()
    ENDIF()
ENDFUNCTION(ADD_GUILE_MODULE2)
