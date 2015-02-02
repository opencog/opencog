
ADD_SUBDIRECTORY (sureal)

# microplanning depends on sureal, so should test after it
ADD_SUBDIRECTORY (microplanning)

IF (HAVE_VITERBI)
	ADD_SUBDIRECTORY (viterbi)
ENDIF (HAVE_VITERBI)
