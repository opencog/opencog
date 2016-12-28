; This rule is for when copula questions, such as "When is the meeting."
; Well actually that's not a copula but let's not get into that, okay.
; You get the idea anyway.
; (AN June 2015)

(define when-cop-q
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
			(dependency "_%atTime" "$verb" "$qVar")
			(dependency "_subj" "$verb" "$subj")
			(Lemma (Variable "$verb") (Word "be"))
			(word-lemma "$subj" "$subj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: whencop-Q-rule")
			(ListLink
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
			)
		)
	)
)
