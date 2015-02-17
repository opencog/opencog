INSTALL (FILES
	apply.scm
	core-docs.scm
	file-utils.scm
	example-dbi.scm
	persistence.scm
	repl-shell.scm
	utilities.scm
	DESTINATION "${DATADIR}/scm"
)
# No CMakeLists.txt in dir:
#ADD_SUBDIRECTORY (wires)
