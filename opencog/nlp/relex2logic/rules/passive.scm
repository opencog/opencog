;check the tense if it is passive
(define (check-tense tense)
	(if (string-contains (cog-name tense) "passive")
		(begin (stv 1 1))
		(begin (stv 0 1))
	)
)

(define passive
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$tense" "DefinedLinguisticConceptNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$obj" "$a-parse")
			(InheritanceLink
				(VariableNode "$verb")
				(VariableNode "$tense")
			)
			(dependency "_obj" "$verb" "$obj")
			(EvaluationLink
				(GroundedPredicateNode "scm: check-tense")
				(ListLink
					(VariableNode "$tense")
				)
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-passive-rule")
			(ListLink
				(VariableNode "$verb")
				(VariableNode "$obj")
			)
		)
	)
)


(define (pre-passive-rule verb obj)
	(passive-rule2
		(cog-name (word-inst-get-lemma verb)) (cog-name verb)
		(cog-name (word-inst-get-lemma obj)) (cog-name obj)
	)
)
