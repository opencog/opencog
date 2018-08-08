; This rule is for which-subjects of prepositional objects, as in
; "Which toy is in this box?"
; (AN June 2015)


(define whichsubjpobjQ
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$prep" "WordInstanceNode")
			(var-decl "$pobj" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$prep-lemma" "WordNode")
			(var-decl "$pobj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$prep" "$a-parse")
			(word-in-parse "$pobj" "$a-parse")
			(dependency "_predadj" "$subj" "$prep")
			(dependency "_pobj" "$prep" "$pobj")
			(dependency "_det" "$subj" "$qVar")
			(word-feat "$qVar" "which")
			(word-lemma "$subj" "$subj-lemma")
			(word-lemma "$prep" "$prep-lemma")
			(word-lemma "$pobj" "$pobj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: whichsubjpobjQ-rule")
			(ListLink
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
				(VariableNode "$prep-lemma")
				(VariableNode "$prep")
				(VariableNode "$pobj-lemma")
				(VariableNode "$pobj")
			)
		)
	)
)
