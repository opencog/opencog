; This the rule for subject-verb-object sentences, such as
; "Johnny ate the dog."
; (AN June 2015)

(define svo
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$W" "WordInstanceNode")
			(var-decl "$X" "WordInstanceNode")
			(var-decl "$Y" "WordInstanceNode")
			(var-decl "$Z" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$verb-lemma" "WordNode")
			(var-decl "$obj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$X" "$a-parse")
			(word-in-parse "$Y" "$a-parse")
			(word-in-parse "$Z" "$a-parse")
			(dependency "_subj" "$Y" "$X")
			(dependency "_obj" "$Y" "$Z")
			(word-lemma "$X" "$subj-lemma")
			(word-lemma "$Y" "$verb-lemma")
			(word-lemma "$Z" "$obj-lemma")

			(AbsentLink
				(dependency "_iobj" "$Y" "$W")
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: SVO-rule")
			(ListLink
				(VariableNode "$subj-lemma")
				(VariableNode "$X")
				(VariableNode "$verb-lemma")
				(VariableNode "$Y")
				(VariableNode "$obj-lemma")
				(VariableNode "$Z")
			)
		)
	)
)
