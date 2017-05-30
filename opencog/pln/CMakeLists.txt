ADD_GUILE_MODULE(FILES
	pln.scm
	focus-set.scm
	utilities.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/pln"
)

ADD_SUBDIRECTORY (rules)
