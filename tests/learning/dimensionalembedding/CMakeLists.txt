IF (HAVE_GUILE)
	ADD_CXXTEST(DimEmbedUTest)
	TARGET_LINK_LIBRARIES(DimEmbedUTest
		dimensionalembedding
		atomspace
		util
		guile
		server
	)
ENDIF (HAVE_GUILE)
