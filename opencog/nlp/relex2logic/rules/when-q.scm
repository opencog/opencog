; This is for when questions with verbs other than "be"
; such as "When did ytou arrive?"
; (AN June 2015)


(define when-q
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$verb-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$qVar" "$a-parse")
			(dependency "_%atTime" "$verb" "$qVar")
			(AbsentLink
				(Lemma (Variable "$verb") (Word "be"))
			)
			(word-lemma "$verb" "$verb-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: when-rule")
			(ListLink
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
			)
		)
	)
)
