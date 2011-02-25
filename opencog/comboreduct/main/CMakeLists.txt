ADD_EXECUTABLE (action-reductor action-reductor.cc)
TARGET_LINK_LIBRARIES(action-reductor
	comboreduct
        util
)

ADD_EXECUTABLE (contin-reductor contin-reductor.cc)
TARGET_LINK_LIBRARIES(contin-reductor
	comboreduct
        util
)

ADD_EXECUTABLE (full-reductor full-reductor.cc)
TARGET_LINK_LIBRARIES(full-reductor
	comboreduct
        util
)

ADD_EXECUTABLE (logical-reductor logical-reductor.cc)
TARGET_LINK_LIBRARIES(logical-reductor
	comboreduct
        util
)

ADD_EXECUTABLE (mixed-reductor mixed-reductor.cc)
TARGET_LINK_LIBRARIES(mixed-reductor
	comboreduct
        util
)

ADD_EXECUTABLE (perception-reductor perception-reductor.cc)
TARGET_LINK_LIBRARIES(perception-reductor
	comboreduct	
        util
)

ADD_EXECUTABLE (interactive-reductor interactive-reductor.cc)
TARGET_LINK_LIBRARIES(interactive-reductor
	comboreduct	
        util
)

ADD_EXECUTABLE (tree_gen tree_gen.cc)
TARGET_LINK_LIBRARIES(tree_gen
	comboreduct
        util
)

ADD_EXECUTABLE (eval-table eval-table.cc)
TARGET_LINK_LIBRARIES(eval-table
    comboreduct
    util
    ${Boost_PROGRAM_OPTIONS_LIBRARY} 
)

# install eval-table
INSTALL(TARGETS eval-table RUNTIME DESTINATION bin)

