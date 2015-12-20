; This rule is for the main predicate in predicate-adjective sentences
; such as "You are very young, sir."
; (AN June 2015)


(define SP
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$predadj" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$predadj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$predadj" "$a-parse")
			(word-lemma "$subj" "$subj-lemma")
			(word-lemma "$predadj" "$predadj-lemma")
			(dependency "_predadj" "$subj" "$predadj")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: SV-rule")
			(ListLink
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
				(VariableNode "$predadj-lemma")
				(VariableNode "$predadj")
			)
		)
	)
)
