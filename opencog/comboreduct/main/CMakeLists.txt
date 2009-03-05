INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}/src/ 
)

LINK_DIRECTORIES(
    ${CMAKE_BINARY_DIR}/src/ComboReduct
)

ADD_EXECUTABLE (action-reductor action-reductor.cc)
TARGET_LINK_LIBRARIES(action-reductor
	ComboReduct
    ${LADSUTIL_LIBRARY}
)

ADD_EXECUTABLE (contin-reductor contin-reductor.cc)
TARGET_LINK_LIBRARIES(contin-reductor
	ComboReduct
    ${LADSUTIL_LIBRARY}
)

ADD_EXECUTABLE (full-reductor full-reductor.cc)
TARGET_LINK_LIBRARIES(full-reductor
	ComboReduct
    ${LADSUTIL_LIBRARY}
)

ADD_EXECUTABLE (logical-reductor logical-reductor.cc)
TARGET_LINK_LIBRARIES(logical-reductor
	ComboReduct
    ${LADSUTIL_LIBRARY}
)

ADD_EXECUTABLE (mixed-reductor mixed-reductor.cc)
TARGET_LINK_LIBRARIES(mixed-reductor
	ComboReduct
    ${LADSUTIL_LIBRARY}
)

ADD_EXECUTABLE (perception-reductor perception-reductor.cc)
TARGET_LINK_LIBRARIES(perception-reductor
	ComboReduct	
    ${LADSUTIL_LIBRARY}
)

ADD_EXECUTABLE (tree_gen tree_gen.cc)
TARGET_LINK_LIBRARIES(tree_gen
	ComboReduct
    ${LADSUTIL_LIBRARY}
)
