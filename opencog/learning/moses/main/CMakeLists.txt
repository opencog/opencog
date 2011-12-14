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

ADD_EXECUTABLE (moses-exec moses-exec)
TARGET_LINK_LIBRARIES (moses-exec
       moses
)
SET_TARGET_PROPERTIES(moses-exec PROPERTIES OUTPUT_NAME "moses")

# install moses-exec
INSTALL(TARGETS moses-exec RUNTIME DESTINATION bin)

#install header files
INSTALL(FILES
    moses_exec.h
    moses_exec_def.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/main" 
)
