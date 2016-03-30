;
; "He stood at the goal line."
(define nn
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$N1" "WordInstanceNode")
			(var-decl "$N2" "WordInstanceNode")
			(var-decl "$N1-lemma" "WordNode")
			(var-decl "$N2-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$N1" "$a-parse")
			(word-in-parse "$N2" "$a-parse")
			(word-lemma "$N1" "$N1-lemma")
			(word-lemma "$N2" "$N2-lemma")
			(dependency "_nn" "$N1" "$N2")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: nn-rule")
			(ListLink
				(VariableNode "$N1-lemma")
				(VariableNode "$N1")
				(VariableNode "$N2-lemma")
				(VariableNode "$N2")
			)
		)
	)
)
