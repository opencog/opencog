
ADD_EXECUTABLE (post_learning-rewriter post_learning-rewriter)
TARGET_LINK_LIBRARIES (post_learning-rewriter
	RewritingRules
	cogutil
)

IF(HAVE_MOSES)
	ADD_EXECUTABLE (moses-petbrain moses-petbrain)
	TARGET_LINK_LIBRARIES (moses-petbrain
		moses
		comboant
		comboreduct
		cogutil
		${Boost_PROGRAM_OPTIONS_LIBRARY}
	)

	ADD_EXECUTABLE (interactive-hillclimbing interactive-hillclimbing)
	TARGET_LINK_LIBRARIES (interactive-hillclimbing
		comboreduct
		comboant
		cogutil
	)
ENDIF(HAVE_MOSES)
