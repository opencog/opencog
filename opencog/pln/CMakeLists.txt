ADD_GUILE_MODULE(FILES
	pln.scm
	focus-set.scm
	utilities.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/pln"
)

ADD_SUBDIRECTORY (rules)
ADD_SUBDIRECTORY (meta-rules)
