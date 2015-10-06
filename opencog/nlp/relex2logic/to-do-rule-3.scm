;"She is nice to help with the project." 
	
(define to-do-rule-3
	(BindLink
		(VariableList
			(TypedVariableLink
				(VariableNode "$a-parse")
				(TypeNode "ParseNode")
			)
			(TypedVariableLink
				(VariableNode "$adj")
				(TypeNode "WordInstanceNode")
			)
			(TypedVariableLink
				(VariableNode "$verb")
				(TypeNode "WordInstanceNode")
			)
		    (TypedVariableLink
				(VariableNode "$subj")
				(TypeNode "WordInstanceNode")
		    )
		)
		(AndLink
			(WordInstanceLink
				(VariableNode "$adj")
				(VariableNode "$a-parse")
			)
			(WordInstanceLink
				(VariableNode "$verb")
				(VariableNode "$a-parse")
			)
		 	(WordInstanceLink
				(VariableNode "$subj")
				(VariableNode "$a-parse")
			)
			(EvaluationLink
				(DefinedLinguisticRelationshipNode "_to-do")
				(ListLink
					(VariableNode "$adj")
					(VariableNode "$verb")
				)
			)
			(EvaluationLink
				(DefinedLinguisticRelationshipNode "_predadj")
				(ListLink
					(VariableNode "$subj")
					(VariableNode "$adj")
				)
			)
		)
   (ListLink
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
)


(define (pre-todo3-rule subj verb adj)
 (ListLink
	(to-do-rule-3 (cog-name (word-inst-get-lemma adj)) (cog-name adj)
		(cog-name (word-inst-get-lemma  verb)) (cog-name verb)
		(cog-name (word-inst-get-lemma  subj)) (cog-name subj)
	)
 )
)
