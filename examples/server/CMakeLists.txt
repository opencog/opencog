ADD_EXECUTABLE (derivedserver
    DerivedCogServer.cc
    DerivedMain.cc
)

TARGET_LINK_LIBRARIES (derivedserver
    server
	boost_signals-mt
    ${CSOCKETS_LIBRARIES}
    ${OPENSSL_LIBRARIES}
)
