ADD_EXECUTABLE (derivedserver
    DerivedCogServer.cc
    DerivedMain.cc
)

TARGET_LINK_LIBRARIES (derivedserver
    server
    ${Boost_SIGNALS_LIBRARY}
)
