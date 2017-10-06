

ADD_LIBRARY (attentionbank
	AVUtils.cc
	AttentionBank.cc
	ImportanceIndex.cc
	StochasticImportanceDiffusion.cc
	ThreadSafeFixedIntegerIndex.cc
)

TARGET_LINK_LIBRARIES(attentionbank
	-Wl,--no-as-needed
	atombase
	truthvalue
	${COGUTIL_LIBRARY}
)

INSTALL (TARGETS attentionbank
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	AVUtils.h
	AttentionBank.h
	ImportanceIndex.h
	StochasticImportanceDiffusion.h
	ThreadSafeFixedIntegerIndex.h
	DESTINATION "include/opencog/attentionbank"
)
