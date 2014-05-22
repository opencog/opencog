IF (HAVE_MOSES)
	ADD_SUBDIRECTORY (clustering)
	ADD_SUBDIRECTORY (moses)
	ADD_SUBDIRECTORY (feature-selection)
ENDIF (HAVE_MOSES)

ADD_SUBDIRECTORY (statistics)

IF (HAVE_DIMEMBED)
    ADD_SUBDIRECTORY (dimensionalembedding)
ENDIF (HAVE_DIMEMBED)

# Currently, the pleasure code does not build, as it uses various
# obsolete combo types and routines and etc.  It needs to be ported
# over to the modern combo interfaces.
#
ADD_SUBDIRECTORY (pleasure)

IF (HAVE_CYTHON)
    ADD_SUBDIRECTORY (pln)
ENDIF (HAVE_CYTHON)

IF (HAVE_ATOMSPACE)
    ADD_SUBDIRECTORY (PatternMiner)
ENDIF (HAVE_ATOMSPACE)

ADD_SUBDIRECTORY (RuleEngine)
