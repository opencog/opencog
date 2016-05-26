IF (HAVE_GUILE)
	ADD_LIBRARY (guile-attention SHARED
		AttentionSCM
	)
	TARGET_LINK_LIBRARIES(guile-attention
		server
		${Boost_SYSTEM_LIBRARY}
		${GUILE_LIBRARIES}
	)

	INSTALL (TARGETS guile-attention DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
        ADD_GUILE_MODULE (attention.scm)
ENDIF (HAVE_GUILE)

