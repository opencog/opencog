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
			(var-decl "$verb-lemma" "WordNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$obj-lemma" "WordNode")
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
			(word-lemma "$verb" "$verb-lemma")
			(word-lemma "$obj" "$obj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: passive-rule2")
			(ListLink
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
				(VariableNode "$obj-lemma")
				(VariableNode "$obj")
			)
		)
	)
)
