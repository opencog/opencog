ADD_CXXTEST(PetUTest)
TARGET_LINK_LIBRARIES(PetUTest 
	OpcLib 
	Procedure 
	behavior 
	PAI 
	Predavese 
	MessagingSystem 
	Control 
        comboreduct
	util 
	pthread 
	ssl
	crypto 
	dl
	${Boost_DATE_TIME_LIBRARY}
	${XERCESC_LIBRARIES}
	${OPENCOG_LIBRARIES}
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(PetUTest winmm)
ENDIF(WIN32)

ADD_EXECUTABLE(HttpRequestTest HttpRequestTest.cc)
TARGET_LINK_LIBRARIES(HttpRequestTest 
	pthread 
	ssl
	crypto 
        dl 
        ${CSOCKETS_LIBRARIES}
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(HttpRequestTest winmm)
ENDIF(WIN32)


ADD_CXXTEST(LanguageComprehensionUTest)
TARGET_LINK_LIBRARIES(LanguageComprehensionUTest 
  OpcLib
  behavior
  ${XERCESC_LIBRARIES}
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(LanguageComprehensionUTest winmm)
ENDIF(WIN32)

