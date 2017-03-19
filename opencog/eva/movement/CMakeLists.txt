INSTALL (FILES
	movement.scm
	DESTINATION "${DATADIR}/scm/opencog/"
)

INSTALL (FILES
	orchestrate.scm
	DESTINATION "${DATADIR}/scm/opencog/movement"
)

INSTALL (FILES
	atomic.py
	atomic-dbg.py
	DESTINATION "${DATADIR}/python/"
)
