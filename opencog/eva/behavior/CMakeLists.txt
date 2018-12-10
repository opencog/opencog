# Install the Eva and Sophia config files; however, the scheme module
# will not load the config parameters -- that has to be done by hand.
ADD_GUILE_MODULE (FILES
	behavior.scm
	express.scm
	eva-behavior.scm
	face-priority.scm
	primitives.scm
	orchestrate.scm
	movement-api.scm
	cfg-tools.scm
	cfg-eva.scm
	cfg-sophia.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/eva-behavior"
)
