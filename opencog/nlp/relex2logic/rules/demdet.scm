; This rule is for demonstrative determiners such as "that bozo" and
; "this job."  It used to be called the det rule but I changed it
; because there are lots of other determiners.
; (AN June 2015)


(define demdet
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$noun" "WordInstanceNode")
			(var-decl "$demdet" "WordInstanceNode")
			(var-decl "$noun-lemma" "WordNode")
			(var-decl "$demdet-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$noun" "$a-parse")
			(word-in-parse "$demdet" "$a-parse")
			(word-lemma "$noun" "$noun-lemma")
			(word-lemma "$demdet" "$demdet-lemma")
			(dependency "_det" "$noun" "$demdet")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: demdet-rule")
			(ListLink
				(VariableNode "$demdet-lemma")
				(VariableNode "$demdet")
				(VariableNode "$noun-lemma")
				(VariableNode "$noun")
			)
		)
	)
)
