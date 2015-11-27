INSTALL (FILES
	relex2logic.scm
	DESTINATION "${DATADIR}/scm/opencog/nlp/"
)

INSTALL (FILES
	post-processing.scm
	rule-helpers.scm
	r2l-utilities.scm
	DESTINATION "${DATADIR}/scm/opencog/nlp/relex2logic"
)

ADD_SUBDIRECTORY (loader)
ADD_SUBDIRECTORY (rules)

