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

ADD_EXECUTABLE (eval-truth-table eval-truth-table)
TARGET_LINK_LIBRARIES (eval-truth-table
        comboreduct
        util
)

ADD_EXECUTABLE (eval-contin-table eval-contin-table)
TARGET_LINK_LIBRARIES (eval-contin-table
        comboreduct
        util
        ${Boost_PROGRAM_OPTIONS_LIBRARY} 
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

ADD_EXECUTABLE (moses-exec moses-exec)
TARGET_LINK_LIBRARIES (moses-exec
       moses
)

# install moses-exec, eval-contin-table
INSTALL(TARGETS moses-exec eval-contin-table RUNTIME DESTINATION bin)

#install header files
INSTALL(FILES
    moses_exec.h
    moses_exec_def.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/main" 
)
