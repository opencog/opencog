ADD_EXECUTABLE (action-reductor action-reductor.cc)
TARGET_LINK_LIBRARIES(action-reductor
	comboreduct
	comboant
	cogutil
)

ADD_EXECUTABLE (contin-reductor contin-reductor.cc)
TARGET_LINK_LIBRARIES(contin-reductor
	comboreduct
	comboant
	cogutil
)

ADD_EXECUTABLE (full-reductor full-reductor.cc)
TARGET_LINK_LIBRARIES(full-reductor
	comboreduct
	comboant
	cogutil
)

ADD_EXECUTABLE (logical-reductor logical-reductor.cc)
TARGET_LINK_LIBRARIES(logical-reductor
	comboreduct
	comboant
	cogutil
)

ADD_EXECUTABLE (mixed-reductor mixed-reductor.cc)
TARGET_LINK_LIBRARIES(mixed-reductor
	comboreduct
	comboant
	cogutil
)

ADD_EXECUTABLE (perception-reductor perception-reductor.cc)
TARGET_LINK_LIBRARIES(perception-reductor
	comboreduct	
	comboant
	cogutil
)

ADD_EXECUTABLE (interactive-reductor interactive-reductor.cc)
TARGET_LINK_LIBRARIES(interactive-reductor
	comboreduct	
	comboant
	cogutil
)

ADD_EXECUTABLE (repl repl.cc)
TARGET_LINK_LIBRARIES(repl
	comboreduct
	comboant
	cogutil
)

ADD_EXECUTABLE (tree_gen tree_gen.cc)
TARGET_LINK_LIBRARIES(tree_gen
	comboreduct
	comboant
	cogutil
)

ADD_EXECUTABLE (tree-sim tree-sim.cc)
TARGET_LINK_LIBRARIES(tree-sim
	comboreduct
	comboant
	cogutil
)

ADD_EXECUTABLE (eval-table eval-table.cc)
TARGET_LINK_LIBRARIES(eval-table
	comboreduct
	comboant
	cogutil
	${Boost_PROGRAM_OPTIONS_LIBRARY} 
)

ADD_EXECUTABLE (gen-table gen-table.cc)
TARGET_LINK_LIBRARIES(gen-table
	comboreduct
	comboant
	cogutil
	${Boost_PROGRAM_OPTIONS_LIBRARY} 
)

ADD_EXECUTABLE (gen-disj-conj gen-disj-conj.cc)
TARGET_LINK_LIBRARIES(gen-disj-conj
	comboreduct
	comboant
	cogutil
	${Boost_PROGRAM_OPTIONS_LIBRARY} 
)

# install eval-table
INSTALL(TARGETS eval-table gen-table gen-disj-conj RUNTIME DESTINATION bin)

