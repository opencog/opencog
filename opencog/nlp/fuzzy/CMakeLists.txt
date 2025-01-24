ADD_LIBRARY (nlpfz SHARED
	Fuzzy.cc
	FuzzyMatch.cc
	FuzzyMatchBasic.cc
	FuzzySCM.cc
)

TARGET_LINK_LIBRARIES (nlpfz
	neighbors
	nlp-types
	nlp-oc-types
	${ATTENTIONBANK_LIBRARIES}
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS nlpfz DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

ADD_GUILE_MODULE (FILES
	fuzzy.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/nlp/fuzzy"
)

ADD_GUILE_EXTENSION(SCM_CONFIG nlpfz "opencog-ext-path-nlp-fuzzy")
