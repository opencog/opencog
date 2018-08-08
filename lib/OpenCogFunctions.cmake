# Copyright (C) 2016 OpenCog Foundation

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
# When cmake is run, a symlink is created at '${CMAKE_BINARY_DIR}/opencog/scm'
# for all the files specified, following the file tree structure created when
# installing to /usr/local/share/opencog/scm. It has two keyword arguments,
#
# FILES: List of files to be installed/symlinked
#
# MODULE_DESTINATION: The absolute path where the files associated
#   with the module are installed, with the exception of the
#   MODULE_FILE(see definition at top of this file). The path for
#   MODULE_FILE, is inferred from this argument, even if it is the only file to
#   be installed.
FUNCTION(ADD_GUILE_MODULE)
    # NOTE: Change PREFIX_DIR_PATH variable if a choice is made to adapt
    # guile's site-package convention.
    SET(PREFIX_DIR_PATH "${DATADIR}/scm")
    SET(options "")  # This is used only as a place-holder
    SET(oneValueArgs MODULE_DESTINATION)
    SET(multiValueArgs FILES)
    CMAKE_PARSE_ARGUMENTS(SCM "${options}" "${oneValueArgs}"
        "${multiValueArgs}" ${ARGN})

    # NOTE:
    # 1. The keyword arguments 'FILES' and 'MODULE_DESTINATION' are required.
    # 2. The keyword argument 'FILES' only works with file-names and not path
    # to files.
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
ENDFUNCTION(ADD_GUILE_MODULE)
