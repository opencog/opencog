ADD_EXECUTABLE (moses-ann moses-ann)
TARGET_LINK_LIBRARIES (moses-ann
	moses
	comboreduct
	util
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

ADD_EXECUTABLE (moses-disjunction moses-disjunction)
TARGET_LINK_LIBRARIES (moses-disjunction
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

ADD_EXECUTABLE (moses-contin-table moses-contin-table)
TARGET_LINK_LIBRARIES (moses-contin-table
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
ADD_EXECUTABLE (gen-contin-table gen-contin-table)
TARGET_LINK_LIBRARIES (gen-contin-table
	moses
        comboreduct
        util
)
ADD_EXECUTABLE (gen-mixed-table gen-mixed-table)
TARGET_LINK_LIBRARIES (gen-mixed-table
	moses
        comboreduct
        util
)
ADD_EXECUTABLE (gen-truth-table gen-truth-table)
TARGET_LINK_LIBRARIES (gen-truth-table
	moses
        comboreduct
        util
)
ADD_EXECUTABLE (moses-disjunction-SA moses-disjunction-SA)
TARGET_LINK_LIBRARIES (moses-disjunction-SA
        moses
        comboreduct
        util
)