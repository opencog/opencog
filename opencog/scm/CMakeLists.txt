INSTALL (FILES
	example-dbi.scm
	load.sh
	type_constructors.scm
	utilities.scm
	DESTINATION "${DATADIR}/scm"
)
ADD_SUBDIRECTORY (wires)
