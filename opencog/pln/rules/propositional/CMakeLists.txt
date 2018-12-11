# Boolean algebra rules
ADD_GUILE_MODULE(FILES
	consequent-disjunction-elimination.scm
	fuzzy-conjunction-introduction.scm
	fuzzy-disjunction-introduction.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/pln/rules/propositional"
)
