ADD_EXECUTABLE (moses-ann-pole2nv moses-ann-pole2nv)
TARGET_LINK_LIBRARIES (moses-ann-pole2nv
	moses
	util
)

ADD_EXECUTABLE (moses-ann-pole1 moses-ann-pole1)
TARGET_LINK_LIBRARIES (moses-ann-pole1
	moses
	util
)

ADD_EXECUTABLE (moses-ann-pole2-hillclimbing moses-ann-pole2-hillclimbing)
TARGET_LINK_LIBRARIES (moses-ann-pole2-hillclimbing
	moses
	util
)

ADD_EXECUTABLE (moses-ann-pole2 moses-ann-pole2)
TARGET_LINK_LIBRARIES (moses-ann-pole2
	moses
	util
)

ADD_EXECUTABLE (moses-ann-xor moses-ann-xor)
TARGET_LINK_LIBRARIES (moses-ann-xor
	moses
	util
)

ADD_EXECUTABLE (continmax continmax)
TARGET_LINK_LIBRARIES (continmax
	moses
	util
)

ADD_EXECUTABLE (nmax nmax)
TARGET_LINK_LIBRARIES (nmax
	moses
	util
)

ADD_EXECUTABLE (onemax onemax)
TARGET_LINK_LIBRARIES (onemax
	moses
	util
)

ADD_EXECUTABLE (ontomax ontomax)
TARGET_LINK_LIBRARIES (ontomax
	moses
	util
)

ADD_EXECUTABLE (trap-bit trap-bit)
TARGET_LINK_LIBRARIES (trap-bit
	moses
	util
)

ADD_EXECUTABLE (trap-uni trap-uni)
TARGET_LINK_LIBRARIES (trap-uni
	moses
	util
)
