INCLUDE_DIRECTORIES (
    ${CMAKE_SOURCE_DIR}/src
)

LINK_DIRECTORIES(
    ${CMAKE_BINARY_DIR}/src/MosesEda
)

ADD_EXECUTABLE (moses-sregression moses-sregression)
TARGET_LINK_LIBRARIES (moses-sregression
	MosesEda
	${COMBOREDUCT_LIBABRY}
	${LADSUTIL_LIBRARY}
)

ADD_EXECUTABLE (build-representation build-representation)
TARGET_LINK_LIBRARIES (build-representation
	MosesEda
	${COMBOREDUCT_LIBABRY}
	${LADSUTIL_LIBRARY}
)

ADD_EXECUTABLE (continmax continmax)
TARGET_LINK_LIBRARIES (continmax
	${COMBOREDUCT_LIBABRY}
	${LADSUTIL_LIBRARY}
	MosesEda
)

ADD_EXECUTABLE (moses-parity moses-parity)
TARGET_LINK_LIBRARIES (moses-parity
	${COMBOREDUCT_LIBABRY}
	${LADSUTIL_LIBRARY}
	MosesEda
)

ADD_EXECUTABLE (nmax nmax)
TARGET_LINK_LIBRARIES (nmax
	${COMBOREDUCT_LIBABRY}
	${LADSUTIL_LIBRARY}
	MosesEda
)

ADD_EXECUTABLE (onemax onemax)
TARGET_LINK_LIBRARIES (onemax
	${COMBOREDUCT_LIBABRY}
	${LADSUTIL_LIBRARY}
	MosesEda
)

ADD_EXECUTABLE (ontomax ontomax)
TARGET_LINK_LIBRARIES (ontomax
	${COMBOREDUCT_LIBABRY}
	${LADSUTIL_LIBRARY}
	MosesEda
)

ADD_EXECUTABLE (moses-ant-hillclimbing moses-ant-hillclimbing)
TARGET_LINK_LIBRARIES (moses-ant-hillclimbing
	${COMBOREDUCT_LIBABRY}
	${LADSUTIL_LIBRARY}
	MosesEda
)

ADD_EXECUTABLE (moses-iterative-hillclimbing moses-iterative-hillclimbing)
TARGET_LINK_LIBRARIES (moses-iterative-hillclimbing
	${COMBOREDUCT_LIBABRY}
	${LADSUTIL_LIBRARY}
	MosesEda
)

ADD_EXECUTABLE (moses-truth-table moses-truth-table)
TARGET_LINK_LIBRARIES (moses-truth-table
	${COMBOREDUCT_LIBABRY}
	${LADSUTIL_LIBRARY}
	MosesEda
)

ADD_EXECUTABLE (eval-truth-table eval-truth-table)
TARGET_LINK_LIBRARIES (eval-truth-table
	${COMBOREDUCT_LIBABRY}
	${LADSUTIL_LIBRARY}
	MosesEda
)
