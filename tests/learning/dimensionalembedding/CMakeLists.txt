IF (HAVE_GUILE)
	ADD_CXXTEST(DimEmbedUTest)
	TARGET_LINK_LIBRARIES(DimEmbedUTest
		dimensionalembedding
		server
		atomspace
		util
		${GUILE_LIBRARIES}
	)
ENDIF (HAVE_GUILE)
