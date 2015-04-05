
# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR})

ADD_LIBRARY (lambda SHARED
	BetaRedex.cc
	BindLink.cc
	DefineLink.cc
	LambdaLink.cc
	PatternUtils.cc
	SatisfactionLink.cc
	VariableList.cc
)

# Without this, parallel make will race and crap up the generated files.
ADD_DEPENDENCIES(lambda opencog_atom_types)

TARGET_LINK_LIBRARIES(lambda
	atomutils
)
