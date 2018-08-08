; This rule processes a relatively new relex relation -- prepositional-adjectival --
; as in "the man in the sombrero" or "the moose under the table"
; (AN June 2015)


(define prepadj
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$noun" "WordInstanceNode")
			(var-decl "$adj" "WordInstanceNode")
			(var-decl "$noun-lemma" "WordNode")
			(var-decl "$adj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$noun" "$a-parse")
			(word-in-parse "$adj" "$a-parse")
			(word-lemma "$noun" "$noun-lemma")
			(word-lemma "$adj" "$adj-lemma")
			(dependency "_prepadj" "$noun" "$adj")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: prepadj-rule")
			(ListLink
				(VariableNode "$noun-lemma")
				(VariableNode "$noun")
				(VariableNode "$adj-lemma")
				(VariableNode "$adj")
			)
		)
	)
)
