INSTALL (FILES
	eva-behavior.scm
	eva-model.scm
	DESTINATION "${DATADIR}/scm/opencog/"
)

# Install the Eva and Sophia config files; however, the scheme module
# will not load the config parameters -- that has to be done by hand.
INSTALL (FILES
	behavior.scm
	express.scm
	primitives.scm
	cfg-tools.scm
	cfg-eva.scm
	cfg-sophia.scm
	DESTINATION "${DATADIR}/scm/opencog/eva-behavior"
)

INSTALL (FILES
	self-model.scm
	faces.scm
	orchestrate.scm
	DESTINATION "${DATADIR}/scm/opencog/eva-model"
)

INSTALL (FILES
	atomic.py
	atomic-dbg.py
	DESTINATION "${DATADIR}/python/"
)
