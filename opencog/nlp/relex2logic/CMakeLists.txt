ADD_GUILE_MODULE (FILES
    relex2logic.scm
    post-processing.scm
    rule-helpers.scm
    rule-utils.scm
    r2l-utilities.scm
    tv-utilities.scm
    MODULE_DESTINATION "${DATADIR}/scm/opencog/nlp/relex2logic"
)

ADD_SUBDIRECTORY (loader)
ADD_SUBDIRECTORY (rules)
