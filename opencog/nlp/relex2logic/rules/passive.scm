; check the tense if it is passive
; XXX Fix relex so that we don't have to make such string searches!
(define-public (check-tense tense)
	(if (string-contains (cog-name tense) "passive") (stv 1 1) (stv 0 1))
)

(define passive
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$tense" "DefinedLinguisticConceptNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$obj" "$a-parse")
			(dependency "_obj" "$verb" "$obj")
			(TenseLink
				(VariableNode "$verb")
				(VariableNode "$tense")
			)
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


(define-public (pre-passive-rule verb obj)
	(passive-rule2
		(cog-name (word-inst-get-lemma verb)) (cog-name verb)
		(cog-name (word-inst-get-lemma obj)) (cog-name obj)
	)
)
