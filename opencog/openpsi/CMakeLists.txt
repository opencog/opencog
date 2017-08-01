ADD_GUILE_MODULE (FILES
	action-selector.scm
	control.scm
	demand.scm
	main.scm
	openpsi.scm
	rule.scm
	utilities.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/openpsi"
)

ADD_SUBDIRECTORY (dynamics)
