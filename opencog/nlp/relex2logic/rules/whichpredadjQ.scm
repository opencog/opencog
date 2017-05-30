; This rule is for which-subjects in predicate-adjective sentences, 
; as in "Which book is better?"
; (AN June 2015)


(define whichpredadjQ
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$predadj" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$predadj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$predadj" "$a-parse")
			(dependency "_predadj" "$subj" "$predadj")
			(dependency "_det" "$subj" "$qVar")
			(word-feat "$qVar" "which")
			(word-lemma "$subj" "$subj-lemma")
			(word-lemma "$predadj" "$predadj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: whichpredadjQ-rule")
			(ListLink
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
				(VariableNode "$preadj-lemma")
				(VariableNode "$predadj")
			)
		)
	)
)
