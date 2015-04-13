IF (HAVE_GUILE)
	ADD_CXXTEST(DimEmbedUTest)
	TARGET_LINK_LIBRARIES(DimEmbedUTest
		dimensionalembedding
		server
		atomspace
		cogutil
		${GUILE_LIBRARIES}
	)
ENDIF (HAVE_GUILE)
