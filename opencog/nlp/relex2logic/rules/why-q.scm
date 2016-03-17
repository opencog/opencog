; This is for why-questions with a verb other than to-be, as in
; "Why did you sleep late?"
; (AN June 2015)

(define why-q
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$verb-lemma" "WordNode")
			(var-decl "$qVar" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$qVar" "$a-parse")
			(dependency "_%because" "$verb" "$qVar")
			(word-lemma "$verb" "$verb-lemma")
			(AbsentLink
				(Lemma (Variable "$verb") (Word "be"))
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: why-rule")
			(ListLink
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
			)
		)
	)
)
