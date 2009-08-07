include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/opencog/atomspace
    ${Boost_INCLUDE_DIR}
    )

add_library(atomspace_py SHARED
    atomspace_module_wrap.cc
    atom_types_wrap.cc
    Atom_wrap.cc
    AtomSpace_wrap.cc
    AtomTable_wrap.cc
    AttentionValue_wrap.cc
    ClassServer_wrap.cc
    CompositeTruthValue_wrap.cc
    CountTruthValue_wrap.cc
	Handle_wrap.cc
	HandleEntry_wrap.cc
    IndefiniteTruthValue_wrap.cc
    Link_wrap.cc
    Node_wrap.cc
    SavableRepository_wrap.cc
    SimpleTruthValue_wrap.cc
    SpaceServer_wrap.cc
    SpaceServerContainer_wrap.cc
    TruthValue_wrap.cc
    TLB_wrap.cc
)
target_link_libraries(atomspace_py
    atomspace
    ${Boost_LIBRARIES}
)
set_target_properties(atomspace_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "libopencog-atomspace-python"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/opencog/python
)
INSTALL (TARGETS atomspace_py
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)
