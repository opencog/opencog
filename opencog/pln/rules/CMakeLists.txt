# Boolean algebra rules
ADD_GUILE_MODULE(FILES
	propositional-logic/fuzzy-conjunction-introduction-rule.scm
	propositional-logic/fuzzy-disjunction-introduction-rule.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/pln/rules"
)

# # Predicate logic rules
# ADD_GUILE_MODULE(FILES
# 	MODULE_DESTINATION "${DATADIR}/scm/opencog/pln/rules/predicate-logic"
# )

# Term logic rules
ADD_GUILE_MODULE(FILES
	term-logic/deduction-rule.scm
	term-logic/formulas.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/pln/rules"
)

# Work in progress rules
ADD_GUILE_MODULE(FILES
	wip/implication-direct-evaluation-rule.scm
	wip/instantiation.scm
	wip/implication-instantiation-rule.scm
	wip/and-introduction-rule.scm
	wip/and-lambda-factorization-double-implication-rule.scm
	wip/and-simplification-rule.scm
	wip/equivalence-scope-distribution-rule.scm
	wip/equivalence-to-implication-rule.scm
	wip/implication-and-lambda-factorization-rule.scm
	wip/implication-implicant-conjunction-rule.scm
	wip/implication-implicant-disjunction-rule.scm
	wip/implication-implicant-distribution-rule.scm
	wip/implication-scope-to-implication-rule.scm
	wip/implication-to-implication-scope-rule.scm
	wip/inversion-rule.scm
	wip/closed-lambda-introduction-rule.scm
	wip/predicate-lambda-evaluation-rule.scm
	wip/and-lambda-distribution-rule.scm
	wip/and-to-context.scm
	wip/contextualize.scm
	wip/extensional-similarity-evaluation-rule.scm
	wip/intensional-inheritance-evaluation-rule.scm
	wip/member-to-evaluation-rule.scm
	wip/or-simplification-rule.scm
	wip/abduction-rule.scm
	wip/and-as-1st-arg-inside-inheritance-link-rule.scm
	wip/and-as-2nd-arg-inside-inheritance-link-rule.scm
	wip/and-breakdown-rule.scm
	wip/and-elimination-rule.scm
	wip/compile-rules.scm
	wip/decontextualize.scm
	wip/evaluation-to-member-rule.scm
	wip/forall-implication-to-higher-order-rule.scm
	wip/forall-instantiation-rule.scm
	wip/implication-and-rule.scm
	wip/induction-rule.scm
	wip/inheritance-to-member-rule.scm
	wip/intensional-similarity-evaluation-rule.scm
	wip/member-to-inheritance-rule.scm
	wip/modus-ponens-rule.scm
	wip/not-introduction-rule.scm
	wip/not-elimination-rule.scm
	wip/not-simplification-rule.scm
	wip/ontological-inheritance-rule.scm
	wip/or-breakdown-rule.scm
	wip/or-introduction-rule.scm
	wip/or-elimination-rule.scm
	wip/or-evaluation-rule.scm
	wip/precise-modus-ponens-rule.scm
	README.md
	wip/subset-evaluation-rule.scm
	wip/symmetric-modus-ponens-rule.scm
	wip/term-probability-rule.scm
	wip/transitive-similarity-rule.scm
	wip/and-evaluation-rule.scm
	wip/and-to-subset-rule1.scm
	wip/and-to-subset-rulen.scm
	wip/and-transformation-rule.scm
	wip/attraction-rule.scm
	wip/context-free-to-sensitive.scm
	wip/evaluation-implication-rule.scm
	wip/implication-introduction-rule.scm
	wip/inheritance-rule.scm
	wip/negated-subset-evaluation-rule.scm
	wip/or-transformation-rule.scm
	wip/similarity-rule.scm
	wip/negation-introduction-rule.scm
	wip/temp-deduction-rule.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/pln/rules"
)
