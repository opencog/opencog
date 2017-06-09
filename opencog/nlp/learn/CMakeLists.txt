
<<<<<<< HEAD
INCLUDE_DIRECTORIES (
	${CMAKE_BINARY_DIR}       # for the NLP atom types
)

ADD_LIBRARY (learn SHARED
	LearnSentence
)

ADD_DEPENDENCIES (learn
	nlp_atom_types            # also include the default types by dependencies
)

TARGET_LINK_LIBRARIES(learn
	nlp-types
	${ICU_LIBRARY}
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS learn DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

ADD_GUILE_MODULE (FILES
	learn.scm
	batch-word-pair.scm
	common.scm
	gram-sim.scm
	link-pipeline.scm
	make-disjuncts.scm
	mst-parser.scm
	pseudo-csets.scm
	singletons.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/nlp/learn"
)
=======
ADD_SUBDIRECTORY (scm)
>>>>>>> opencog/master
