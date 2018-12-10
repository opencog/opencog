ADD_SUBDIRECTORY(procedures)

ADD_GUILE_MODULE (FILES
	ghost.scm
	cs-parse.scm
	terms.scm
	translator.scm
	matcher.scm
	stimulation.scm
	utils.scm
	test.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/ghost"
)
