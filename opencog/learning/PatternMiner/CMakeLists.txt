# Pattern miner build file.

IF (HAVE_cpprest)
    ADD_LIBRARY (PatternMiner SHARED
            HTree
            PatternMiner
            PatternMinerBF
            PatternMinerDF
            PatternMinerDistributedWorker
            PatternMinerCentralServer
    )

    TARGET_LINK_LIBRARIES (PatternMiner
        ${COGUTIL_LIBRARY}
        ${ATOMSPACE_LIBRARIES}
        ${cpprest_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        ssl
        crypto
    )
ELSE (HAVE_cpprest)
    ADD_LIBRARY (PatternMiner SHARED
            HTree
            PatternMiner
            PatternMinerBF
            PatternMinerDF
    )

    TARGET_LINK_LIBRARIES (PatternMiner
        ${COGUTIL_LIBRARY}
        ${ATOMSPACE_LIBRARIES}
        ${Boost_SYSTEM_LIBRARY}
        ssl
        crypto
    )
ENDIF (HAVE_cpprest)

ADD_DEPENDENCIES(PatternMiner embodiment_atom_types)
ADD_DEPENDENCIES(PatternMiner spacetime_atom_types)


ADD_LIBRARY(PatternMinerAgent SHARED
        PatternMinerAgent
)


TARGET_LINK_LIBRARIES (PatternMinerAgent
	PatternMiner
)

INSTALL (FILES
        HTree.h
        PatternMiner.h
        DESTINATION "include/${PROJECT_NAME}/learning/PatternMiner"
)

IF (HAVE_cpprest)

    ADD_LIBRARY(DistributedPatternMinerClient SHARED
            DistributedPatternMinerClient
    )

    ADD_LIBRARY(DistributedPatternMinerServer SHARED
            DistributedPatternMinerServer
    )

    TARGET_LINK_LIBRARIES (DistributedPatternMinerClient
            PatternMiner
    )

    TARGET_LINK_LIBRARIES (DistributedPatternMinerServer
            PatternMiner
    )

    INSTALL (FILES
            DistributedPatternMiner.h
            DESTINATION "include/${PROJECT_NAME}/learning/PatternMiner"
    )

ENDIF (HAVE_cpprest)




