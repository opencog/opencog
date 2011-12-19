ADD_EXECUTABLE (moses-ann-pole2nv moses-ann-pole2nv)
TARGET_LINK_LIBRARIES (moses-ann-pole2nv
        moses
        comboreduct
        util
)

ADD_EXECUTABLE (moses-ann-pole1 moses-ann-pole1)
TARGET_LINK_LIBRARIES (moses-ann-pole1
        moses
        comboreduct
        util
)

ADD_EXECUTABLE (moses-ann-pole2-hillclimbing moses-ann-pole2-hillclimbing)
TARGET_LINK_LIBRARIES (moses-ann-pole2-hillclimbing
        moses
        comboreduct
        util
)

ADD_EXECUTABLE (moses-ann-pole2 moses-ann-pole2)
TARGET_LINK_LIBRARIES (moses-ann-pole2
        moses
        comboreduct
        util
)

ADD_EXECUTABLE (moses-ann-xor moses-ann-xor)
TARGET_LINK_LIBRARIES (moses-ann-xor
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
