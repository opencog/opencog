

ADD_LIBRARY (attentionbank
	AttentionBank.cc
	ImportanceIndex.cc
	StochasticImportanceDiffusion.cc
)

TARGET_LINK_LIBRARIES(attentionbank
	-Wl,--no-as-needed
	atombase
	truthvalue
	${COGUTIL_LIBRARY}
	${Boost_THREAD_LIBRARY}
)

INSTALL (TARGETS attentionbank
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	AttentionBank.h
	ImportanceIndex.h
	StochasticImportanceDiffusion.h
	DESTINATION "include/opencog/attentionbank"
)
