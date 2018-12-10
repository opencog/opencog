ADD_GUILE_MODULE (FILES
    cogserver.scm
    MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/cogserver"
)

# NOTE: config.scm shouldn't be installed it is only for use during development.
