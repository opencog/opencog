INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY (OpenPsi SHARED
	OpenPsiModule
	PsiDemandUpdaterAgent
	PsiActionSelectionAgent
	PsiRuleUtil
	PsiModulatorUpdaterAgent
)
