
# Build C++ code in assorted NLP subdirectories

ADD_SUBDIRECTORY (chatbot)

# The code in the question dir is now deprecated,
# and should not be built
# ADD_SUBDIRECTORY (question)

ADD_SUBDIRECTORY (types)
ADD_SUBDIRECTORY (wsd)

IF (HAVE_VITERBI)
	# Currently does not build, due to changes in SchemeEval API.
	# ADD_SUBDIRECTORY (viterbi)
ENDIF (HAVE_VITERBI)
