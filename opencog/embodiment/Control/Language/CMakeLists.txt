ADD_LIBRARY (Language SHARED
    LanguageComprehension
    FramesToRelexRuleEngine
    NLGenClient
    OutputRelex
)

TARGET_LINK_LIBRARIES (Language
	atomspace
)

