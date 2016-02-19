; This rule is for where questions with verb "be"
; such as "Where is the meeting?"
;(AN June 2015)

(define where-cop-q
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
			(dependency "_%atLocation" "$verb" "$qVar")
			(dependency "_subj" "$verb" "$subj")
			(Lemma (Variable "$verb") (Word "be"))
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: wherecop-Q-rule")
			(ListLink
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
			)
		)
	)
)
