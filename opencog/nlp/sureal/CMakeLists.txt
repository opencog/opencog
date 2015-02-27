INCLUDE_DIRECTORIES (
    ${CMAKE_BINARY_DIR}       # for the NLP atom types
)

ADD_LIBRARY (sureal SHARED
    SuRealSCM
    SuRealPMCB
)

ADD_DEPENDENCIES (sureal
    nlp_atom_types            # also include the default types by dependencies
)

TARGET_LINK_LIBRARIES(sureal
    atomspace
    query
    smob
)

ADD_LIBRARY (SuRealModule SHARED
    SuRealModule
)

ADD_DEPENDENCIES (SuRealModule
    nlp_atom_types
)

TARGET_LINK_LIBRARIES(SuRealModule
    sureal
)
