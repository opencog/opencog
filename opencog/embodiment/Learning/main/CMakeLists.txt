
ADD_EXECUTABLE (post_learning-rewriter post_learning-rewriter)
TARGET_LINK_LIBRARIES (post_learning-rewriter
	RewritingRules
	${COGUTIL_LIBRARY}
	${Boost_REGEX_LIBRARY}
)

IF(HAVE_MOSES)
	ADD_EXECUTABLE (moses-petbrain moses-petbrain)
	TARGET_LINK_LIBRARIES (moses-petbrain
		moses
		comboant
		comboreduct
		mosesutil
		${COGUTIL_LIBRARY}
		${Boost_PROGRAM_OPTIONS_LIBRARY}
	)

	ADD_EXECUTABLE (interactive-hillclimbing interactive-hillclimbing)
	TARGET_LINK_LIBRARIES (interactive-hillclimbing
		comboreduct
		comboant
		${COGUTIL_LIBRARY}
	)
ENDIF(HAVE_MOSES)
