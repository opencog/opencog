# Boolean algebra rules
ADD_GUILE_MODULE(FILES
	propositional/fuzzy-conjunction-introduction.scm
	propositional/fuzzy-disjunction-introduction.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/pln/rules"
)

# # Predicate logic rules
# ADD_GUILE_MODULE(FILES
# 	MODULE_DESTINATION "${DATADIR}/scm/opencog/pln/rules/predicate"
# )

# Term logic rules
ADD_GUILE_MODULE(FILES
	term/deduction.scm
	term/formulas.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/pln/rules"
)

# Work in progress rules
ADD_GUILE_MODULE(FILES
	wip/implication-direct-evaluation.scm
	wip/instantiation.scm
	wip/implication-instantiation.scm
	wip/and-introduction.scm
	wip/and-lambda-factorization-double-implication.scm
	wip/and-simplification.scm
	wip/equivalence-scope-distribution.scm
	wip/equivalence-to-implication.scm
	wip/implication-and-lambda-factorization.scm
	wip/implication-implicant-conjunction.scm
	wip/implication-implicant-disjunction.scm
	wip/implication-implicant-distribution.scm
	wip/implication-scope-to-implication.scm
	wip/implication-to-implication-scope.scm
	wip/inversion.scm
	wip/closed-lambda-introduction.scm
	wip/predicate-lambda-evaluation.scm
	wip/and-lambda-distribution.scm
	wip/and-to-context.scm
	wip/contextualize.scm
	wip/extensional-similarity-evaluation.scm
	wip/intensional-inheritance-evaluation.scm
	wip/member-to-evaluation.scm
	wip/or-simplification.scm
	wip/abduction.scm
	wip/and-as-1st-arg-inside-inheritance-link.scm
	wip/and-as-2nd-arg-inside-inheritance-link.scm
	wip/and-breakdown.scm
	wip/and-elimination.scm
	wip/compile-rules.scm
	wip/decontextualize.scm
	wip/evaluation-to-member.scm
	wip/forall-implication-to-higher-order.scm
	wip/forall-instantiation.scm
	wip/implication-and.scm
	wip/induction.scm
	wip/inheritance-to-member.scm
	wip/intensional-similarity-evaluation.scm
	wip/member-to-inheritance.scm
	wip/modus-ponens.scm
	wip/not-introduction.scm
	wip/not-elimination.scm
	wip/not-simplification.scm
	wip/ontological-inheritance.scm
	wip/or-breakdown.scm
	wip/or-introduction.scm
	wip/or-elimination.scm
	wip/or-evaluation.scm
	wip/precise-modus-ponens.scm
	README.md
	wip/subset-evaluation.scm
	wip/symmetric-modus-ponens.scm
	wip/term-probability.scm
	wip/transitive-similarity.scm
	wip/and-evaluation.scm
	wip/and-to-subset-rule1.scm
	wip/and-to-subset-rulen.scm
	wip/and-transformation.scm
	wip/attraction.scm
	wip/context-free-to-sensitive.scm
	wip/evaluation-implication.scm
	wip/implication-introduction.scm
	wip/inheritance.scm
	wip/negated-subset-evaluation.scm
	wip/or-transformation.scm
	wip/similarity.scm
	wip/negation-introduction.scm
	wip/temp-deduction.scm
	wip/formulas.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/pln/rules"
)
