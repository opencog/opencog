;"She is nice to help with the project."

(define todo3
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$adj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$verb-lemma" "WordNode")
			(var-decl "$adj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$adj" "$a-parse")
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$subj" "$a-parse")
			(dependency "_to-do" "$adj" "$verb")
			(dependency "_predadj" "$subj" "$adj")
			(word-lemma "$subj" "$subj-lemma")
			(word-lemma "$verb" "$verb-lemma")
			(word-lemma "$adj" "$adj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: to-do-rule-3")
			(ListLink
				(VariableNode "$adj-lemma")
				(VariableNode "$adj")
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
			)
		)
   )
)
