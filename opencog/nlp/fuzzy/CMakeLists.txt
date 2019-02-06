ADD_LIBRARY (nlpfz SHARED
	Fuzzy
	FuzzyMatch
	FuzzyMatchBasic
	FuzzySCM
)

TARGET_LINK_LIBRARIES (nlpfz
	neighbors
	nlp-types
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS nlpfz DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

ADD_GUILE_MODULE (FILES
	fuzzy.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/nlp/fuzzy"
)
