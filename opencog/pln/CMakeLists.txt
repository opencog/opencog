INSTALL (FILES
	rules/implication-direct-evaluation-rule.scm
	rules/instantiation.scm
	rules/implication-instantiation-rule.scm
	rules/and-construction-rule.scm
	rules/and-lambda-factorization-double-implication-rule.scm
	rules/and-simplification-rule.scm
	rules/deduction-rule.scm
	rules/equivalence-scope-distribution-rule.scm
	rules/equivalence-to-double-implication-rule.scm
	rules/formulas.scm
	rules/implication-and-lambda-factorization-rule.scm
	rules/implication-implicant-conjunction-rule.scm
	rules/implication-implicant-disjunction-rule.scm
	rules/implication-implicant-distribution-rule.scm
	rules/implication-scope-distribution-rule.scm
	rules/implication-scope-factorization-rule.scm
	rules/inversion-rule.scm
	rules/lambda-grounded-construction-rule.scm
	rules/lambda-predicate-construction-rule.scm
	rules/and-lambda-distribution-rule.scm
	rules/and-to-context.scm
	rules/contextualize.scm
	rules/extensional-similarity-evaluation-rule.scm
	rules/intensional-inheritance-evaluation-rule.scm
	rules/member-to-evaluation-rule.scm
	rules/or-simplification-rule.scm
	rules/abduction-rule.scm
	rules/and-as-1st-arg-inside-inheritance-link-rule.scm
	rules/and-as-2nd-arg-inside-inheritance-link-rule.scm
	rules/and-breakdown-rule.scm
	rules/and-elimination-rule.scm
	rules/compile-rules.scm
	rules/decontextualize.scm
	rules/evaluation-to-member-rule.scm
	rules/forall-implication-to-higher-order-rule.scm
	rules/forall-instantiation-rule.scm
	rules/implication-and-rule.scm
	rules/induction-rule.scm
	rules/inheritance-to-member-rule.scm
	rules/intensional-similarity-evaluation-rule.scm
	rules/member-to-inheritance-rule.scm
	rules/modus-ponens-rule.scm
	rules/not-construction-rule.scm
	rules/not-elimination-rule.scm
	rules/not-simplification-rule.scm
	rules/ontological-inheritance-rule.scm
	rules/or-breakdown-rule.scm
	rules/or-construction-rule.scm
	rules/or-elimination-rule.scm
	rules/or-evaluation-rule.scm
	rules/precise-modus-ponens-rule.scm
	rules/README.md
	rules/subset-evaluation-rule.scm
	rules/symmetric-modus-ponens-rule.scm
	rules/term-probability-rule.scm
	rules/transitive-similarity-rule.scm
	rules/and-evaluation-rule.scm
	rules/and-to-subset-rule1.scm
	rules/and-to-subset-rulen.scm
	rules/and-transformation-rule.scm
	rules/attraction-rule.scm
	rules/context-free-to-sensitive.scm
	rules/evaluation-implication-rule.scm
	rules/implication-construction-rule.scm
	rules/inheritance-rule.scm
	rules/negated-subset-evaluation-rule.scm
	rules/or-transformation-rule.scm
	rules/similarity-rule.scm
	DESTINATION "${DATADIR}/scm/opencog/pln/rules"
)

# TODO: The following rules are temporary hacks; remove them as the bugs are
# fixed.
INSTALL (FILES
	rules/temp-deduction-rule.scm
	DESTINATION "${DATADIR}/scm/opencog/pln/rules"
)
