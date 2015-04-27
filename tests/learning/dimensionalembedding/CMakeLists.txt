IF (HAVE_GUILE)
	ADD_CXXTEST(DimEmbedUTest)
	TARGET_LINK_LIBRARIES(DimEmbedUTest
		dimensionalembedding
		server
		atomspace
		${COGUTIL_LIBRARY}
		${GUILE_LIBRARIES}
	)
ENDIF (HAVE_GUILE)
