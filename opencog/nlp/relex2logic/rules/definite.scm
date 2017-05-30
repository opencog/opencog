; This rule simply inherits the linguistic concept of being definite to
; any definite noun such as "that man."
; (AN June 2015)

; Names of things (G and AN links) will get concatenated by Relex,
; leaving some of the words that make up the name without a lemma.
; Ignore those.  This is fixed in opencog/relex/pull/243 and so the
; code below can be removed after a few months -- say, after September
; 2016 --  we leave it in as a backwards compat sop for now.
(define-public (r2l-check-name lemma)
	(if (equal? "" (cog-name lemma))
		(stv 0 1)
		(stv 1 1)
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
				(GroundedPredicateNode "scm: r2l-check-name")
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
