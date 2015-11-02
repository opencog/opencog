INSTALL (FILES
	post-processing.scm
	rule-helpers.scm
	utilities.scm
	DESTINATION "${DATADIR}/scm/nlp/relex2logic"
)

ADD_SUBDIRECTORY (loader)
ADD_SUBDIRECTORY (rules)

