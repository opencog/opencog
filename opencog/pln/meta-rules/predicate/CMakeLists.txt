# Predicate logic meta-rules
ADD_GUILE_MODULE(FILES
	conditional-full-instantiation.scm
	conditional-partial-instantiation.scm
	universal-full-instantiation.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/pln/meta-rules/predicate"
)
