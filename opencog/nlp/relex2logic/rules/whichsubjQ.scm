; This rule is for which-subjects of SVO sentences, as in
; "Which guy ate all the pizza?"
; (AN June 2015)

(define whichsubjQ
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$verb-lemma" "WordNode")
			(var-decl "$obj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$obj" "$a-parse")
			(dependency "_subj" "$verb" "$subj")
			(dependency "_obj" "$verb" "$obj")
			(dependency "_det" "$subj" "$qVar")
			(word-feat "$qVar" "which")
			(word-lemma "$subj" "$subj-lemma")
			(word-lemma "$verb" "$verb-lemma")
			(word-lemma "$obj" "$obj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: whichsubjQ-rule")
			(ListLink
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
				(VariableNode "$obj-lemma")
				(VariableNode "$obj")
			)
		)
	)
)
