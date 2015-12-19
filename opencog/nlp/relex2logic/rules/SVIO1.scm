; This rule is for sentences with an indirect object, such as:
; "I gave the dog some LSD."  "I sang a song to her."
; For indirect objects hanging off a preposition, see SVIO2.
; (AN June 2015)


(define SVIO1
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$iobj" "WordInstanceNode")
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
			(word-lemma "$subj" "$subj-lemma")
			(word-lemma "$verb" "$verb-lemma")
			(word-lemma "$obj" "$obj-lemma")
			(word-lemma "$iobj" "$iobj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: SVIO-rule")
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
