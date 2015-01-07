INCLUDE_DIRECTORIES (
    ${CMAKE_BINARY_DIR}       # for the NLP atom types
)

ADD_LIBRARY (sureal SHARED
    SuRealPMCB
)

ADD_DEPENDENCIES (sureal
    nlp_atom_types            # also include the default types by dependencies
)

ADD_LIBRARY (SuRealModule SHARED
    SuRealModule
)

ADD_DEPENDENCIES (SuRealModule
    nlp_atom_types
)

TARGET_LINK_LIBRARIES(SuRealModule
    atomspace
    query
    smob
    sureal
)
