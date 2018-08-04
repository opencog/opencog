; This rule is for which objects in indirect object sentences,
; such as "Which book did you give to your friend?"
; (AN June 2015)

(define whichobjSVIOQ
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$iobj" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$verb-lemma" "WordNode")
			(var-decl "$obj-lemma" "WordNode")
			(var-decl "$iobj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$obj" "$a-parse")
			(word-in-parse "$iobj" "$a-parse")
			(dependency "_subj" "$verb" "$subj")
			(dependency "_obj" "$verb" "$obj")
			(dependency "_iobj" "$verb" "$iobj")
			(dependency "_det" "$obj" "$qVar")
			(word-feat "$qVar" "which")
			(word-lemma "$subj" "$subj-lemma")
			(word-lemma "$verb" "$verb-lemma")
			(word-lemma "$obj" "$obj-lemma")
			(word-lemma "$iobj" "$iobj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: whichobjSVIOQ-rule")
			(ListLink
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
				(VariableNode "$obj-lemma")
				(VariableNode "$obj")
				(VariableNode "$iobj-lemma")
				(VariableNode "$iobj")
			)
		)
	)
)
