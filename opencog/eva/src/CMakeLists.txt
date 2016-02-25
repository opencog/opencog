INSTALL (FILES
	eva-model.scm
	DESTINATION "${DATADIR}/scm/opencog/"
)

INSTALL (FILES
	self-model.scm
	faces.scm
	orchestrate.scm
	DESTINATION "${DATADIR}/scm/opencog/eva-model"
)

INSTALL (FILES
	atomic-dbg.py
	DESTINATION "${DATADIR}/python/"
)
