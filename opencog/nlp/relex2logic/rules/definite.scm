; This rule simply inherits the linguistic concept of being definite to
; any definite noun such as "that man."
; (AN June 2015)

; Names of things (AN links) will get concatenated by Relex,
; leaving some of the words that make up the name without a lemma.
; Ignore those.
(define-public (check-name lemma)
	(if (not (equal? "" (cog-name lemma)))
		(stv 1 1)
		(stv 0 1)
	)
)

(define definite
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$noun" "WordInstanceNode")
			(var-decl "$lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$noun" "$a-parse")
			(word-lemma "$noun" "$lemma")
			(word-feat "$noun" "definite")
			(EvaluationLink
				(GroundedPredicateNode "scm: check-name")
				(ListLink
					(VariableNode "$lemma")
				)
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: definite-rule")
			(ListLink
				(VariableNode "$lemma")
				(VariableNode "$noun"))
		)
	)
)
