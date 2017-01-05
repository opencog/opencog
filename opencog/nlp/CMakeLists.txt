
# Build C++ code in assorted NLP subdirectories

ADD_SUBDIRECTORY (irc)
ADD_SUBDIRECTORY (learn)
ADD_SUBDIRECTORY (scm)
ADD_SUBDIRECTORY (types)
ADD_SUBDIRECTORY (wsd)

# NOTE: Set BUILD_LOJBAN variable by running `cmake -DBUILD_LOJBAN=1 ..` from
# the build directory.
IF (HAVE_STACK AND BUILD_LOJBAN)
	ADD_SUBDIRECTORY (lojban)
ENDIF (HAVE_STACK AND BUILD_LOJBAN)

IF (HAVE_NLP)
	ADD_SUBDIRECTORY (relex2logic)
	ADD_SUBDIRECTORY (aiml)
	ADD_SUBDIRECTORY (chatbot)
	ADD_SUBDIRECTORY (chatbot-eva)
	ADD_SUBDIRECTORY (chatbot-psi)
	# fuzzy depends on sureal
	ADD_SUBDIRECTORY (fuzzy)
	ADD_SUBDIRECTORY (lg-dict)
	ADD_SUBDIRECTORY (microplanning)
	ADD_SUBDIRECTORY (sureal)
	ADD_SUBDIRECTORY (viterbi)
ENDIF (HAVE_NLP)
