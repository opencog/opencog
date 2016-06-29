; This rule is for which-prepositional objects, as in
; "Which box are the drugs in?" or "In which way does this make sense?"
; (AN June 2015)

(define whichpobjQ
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$prep" "WordInstanceNode")
			(var-decl "$pobj" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$be" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$prep-lemma" "WordNode")
			(var-decl "$pobj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$prep" "$a-parse")
			(word-in-parse "$pobj" "$a-parse")
			(dependency "_subj" "$be" "$subj")
			(dependency "_obj" "$prep" "$pobj")
			(dependency "_advmod" "$be" "$prep")
			(dependency "_det" "$pobj" "$qVar")
			(word-feat "$qVar" "which")
			(word-lemma "$subj" "$subj-lemma")
			(word-lemma "$prep" "$prep-lemma")
			(word-lemma "$pobj" "$pobj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: whichpobjQ-rule")
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
