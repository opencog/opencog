ADD_GUILE_MODULE (FILES
	openpsi.scm
	action-selector.scm
	main.scm
	rule.scm
	utilities.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/openpsi"
)

ADD_SUBDIRECTORY (dynamics)

ADD_LIBRARY (openpsi SHARED
	OpenPsiSatisfier.cc
	OpenPsiImplicator.cc
	OpenPsiRules.cc
	OpenPsiSCM.cc
)

TARGET_LINK_LIBRARIES (openpsi
	${ATOMSPACE_LIBRARIES}
)

ADD_GUILE_EXTENSION(SCM_CONFIG openpsi "opencog-ext-path-openpsi")

INSTALL (TARGETS openpsi DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

INSTALL (FILES
	OpenPsiImplicator.h
	OpenPsiSatisfier.h
	DESTINATION "include/opencog/openpsi/"
)
