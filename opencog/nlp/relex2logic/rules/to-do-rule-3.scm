;"She is nice to help with the project."

(define to-do-rule-3
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$adj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$subj" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$adj" "$a-parse")
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$subj" "$a-parse")
			(dependency "_to-do" "$adj" "$verb")
			(dependency "_predadj" "$subj" "$adj")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-todo3-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$verb")
			 	(VariableNode "$adj")
			)
		)
   )
)


(define (pre-todo3-rule subj verb adj)
	(to-do-rule-3 (cog-name (word-inst-get-lemma adj)) (cog-name adj)
		(cog-name (word-inst-get-lemma  verb)) (cog-name verb)
		(cog-name (word-inst-get-lemma  subj)) (cog-name subj)
	)
)
