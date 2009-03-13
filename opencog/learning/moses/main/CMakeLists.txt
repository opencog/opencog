INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/
    ${OPENCOG_SOURCE_DIR}/opencog/learning/
    ${OPENCOG_SOURCE_DIR}/opencog/learning/moses
)

LINK_DIRECTORIES(
    ${OPENCOG_BINARY_DIR}/opencog/learning/moses
    ${OPENCOG_BINARY_DIR}/opencog/util
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
)

ADD_EXECUTABLE (moses-sregression moses-sregression)
TARGET_LINK_LIBRARIES (moses-sregression
	moses
        comboreduct
        util
)

ADD_EXECUTABLE (build-representation build-representation)
TARGET_LINK_LIBRARIES (build-representation
	moses
        comboreduct
        util
)

ADD_EXECUTABLE (continmax continmax)
TARGET_LINK_LIBRARIES (continmax
	moses
        comboreduct
        util
)

ADD_EXECUTABLE (moses-parity moses-parity)
TARGET_LINK_LIBRARIES (moses-parity
	moses
        comboreduct
        util
)

ADD_EXECUTABLE (nmax nmax)
TARGET_LINK_LIBRARIES (nmax
	moses
        comboreduct
        util
)

ADD_EXECUTABLE (onemax onemax)
TARGET_LINK_LIBRARIES (onemax
	moses
        comboreduct
        util
)

ADD_EXECUTABLE (ontomax ontomax)
TARGET_LINK_LIBRARIES (ontomax
	moses
        comboreduct
        util
)

ADD_EXECUTABLE (moses-ant-hillclimbing moses-ant-hillclimbing)
TARGET_LINK_LIBRARIES (moses-ant-hillclimbing
	moses
        comboreduct
        util
)

ADD_EXECUTABLE (moses-iterative-hillclimbing moses-iterative-hillclimbing)
TARGET_LINK_LIBRARIES (moses-iterative-hillclimbing
	moses
        comboreduct
        util
)

ADD_EXECUTABLE (moses-truth-table moses-truth-table)
TARGET_LINK_LIBRARIES (moses-truth-table
	moses
        comboreduct
        util
)

ADD_EXECUTABLE (eval-truth-table eval-truth-table)
TARGET_LINK_LIBRARIES (eval-truth-table
	moses
        comboreduct
        util
)
