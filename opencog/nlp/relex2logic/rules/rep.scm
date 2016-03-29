;
; This rule is for sentential complements following propositional
; attitude verbs, such as "I know that you lied to me." or "I imagine
; you will make a fine garbage man someday, son." It hooks up `know` to
; `that`, and `imagine` to `will` in the sentences above.  This relation
; exists so that someday OpenCog will understand that the situations
; described in the sentences following propositional attitude verbs must
; be interpreted as 'representations' rather than 'reality' which means
; that you can't infer anything about reality from them.  Look up
; Jackendoff's writing on this subject if you wish to understand more . . .
; (AN June 2015)


(define rep
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$pred" "WordInstanceNode")
			(var-decl "$comp" "WordInstanceNode")
			(var-decl "$pred-lemma" "WordNode")
			(var-decl "$comp-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$pred" "$a-parse")
			(word-in-parse "$comp" "$a-parse")
			(word-lemma "$pred" "$pred-lemma")
			(word-lemma "$comp" "$comp-lemma")
			(dependency "_rep" "$pred" "$comp")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: rep-rule")
			(ListLink
				(VariableNode "$comp-lemma")
				(VariableNode "$comp")
				(VariableNode "$pred-lemma")
				(VariableNode "$pred")
			)
		)
	)
)
