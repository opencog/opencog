ADD_GUILE_MODULE (FILES
	action-selector.scm
	main.scm
	openpsi.scm
	rule.scm
	utilities.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/openpsi"
)

ADD_SUBDIRECTORY (dynamics)

ADD_LIBRARY (openpsi SHARED
	OpenPsiImplicator.cc
	OpenPsiRules.cc
	OpenPsiSCM.cc
)

TARGET_LINK_LIBRARIES (openpsi
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS openpsi DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

INSTALL (FILES OpenPsiImplicator.h
	DESTINATION "include/opencog/openpsi/"
)
