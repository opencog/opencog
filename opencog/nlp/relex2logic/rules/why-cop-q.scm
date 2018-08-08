; This is for why-questions with the verb to-be, such as
; "Why are you such a scmhuck?"
; (AN June 2015)


(define why-cop-q
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$qVar" "$a-parse")
			(word-in-parse "$subj" "$a-parse")
			(word-lemma "$subj" "$subj-lemma")
			(dependency "_%because" "$verb" "$qVar")
			(ChoiceLink
				(AndLink
					(dependency "_subj" "$verb" "$subj")
					(Lemma (Variable "$verb") (Word "be"))
				)
				(dependency "_predadj" "$verb" "$subj")
				(dependency "_psubj" "$subj" "$verb")
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: whycop-Q-rule")
			(ListLink
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
			)
		)
	)
)
