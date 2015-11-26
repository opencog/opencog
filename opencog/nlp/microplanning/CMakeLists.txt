
INSTALL (FILES
	microplanning.scm
	DESTINATION "${DATADIR}/scm/opencog/nlp"
)

INSTALL (FILES
	main.scm
	anaphora-noun-item.scm
	chunks-option.scm
	anaphora-nouns-list.scm
	chunks-set.scm
	sentence-forms.scm
	anaphora.scm
	helpers.scm
	atomW.scm

	DESTINATION "${DATADIR}/scm/opencog/nlp/microplanning"
)
