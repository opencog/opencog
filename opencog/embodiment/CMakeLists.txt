
IF (HAVE_ATOMSPACE)
	# We build this unconditionally, in order to get the atom types,
	# which cython needs (cython can't dynamically load any modules,
	# and so must static link them at compile time.)
	ADD_SUBDIRECTORY (AtomSpaceExtensions)
ENDIF (HAVE_ATOMSPACE)

IF (HAVE_EMBODIMENT)
	ADD_SUBDIRECTORY (AutomatedSystemTest)
	ADD_SUBDIRECTORY (Control)
	ADD_SUBDIRECTORY (Learning)
	ADD_SUBDIRECTORY (WorldWrapper)
	ADD_SUBDIRECTORY (AvatarComboVocabulary)
ENDIF (HAVE_EMBODIMENT)
