
# Build C++ code in assorted NLP subdirectories

ADD_SUBDIRECTORY (irc)
ADD_SUBDIRECTORY (learn)
ADD_SUBDIRECTORY (relex2logic)
ADD_SUBDIRECTORY (scm)
ADD_SUBDIRECTORY (types)
ADD_SUBDIRECTORY (wsd)

IF (HAVE_GUILE AND HAVE_LINK_GRAMMAR)
	ADD_SUBDIRECTORY (chatbot)
	# fuzzy depends on sureal
	ADD_SUBDIRECTORY (fuzzy)
	ADD_SUBDIRECTORY (lg-dict)
	ADD_SUBDIRECTORY (sureal)
ENDIF (HAVE_GUILE AND HAVE_LINK_GRAMMAR)

IF (HAVE_VITERBI)
	ADD_SUBDIRECTORY (viterbi)
ENDIF (HAVE_VITERBI)

INSTALL (FILES
	microplanning/microplanning.scm
	microplanning/anaphora-noun-item.scm
	microplanning/chunks-option.scm
	microplanning/anaphora-nouns-list.scm
	microplanning/chunks-set.scm
	microplanning/sentence-forms.scm
	microplanning/anaphora.scm
	microplanning/helpers.scm
	microplanning/atomW.scm

	DESTINATION "${DATADIR}/scm/opencog/nlp"
)
