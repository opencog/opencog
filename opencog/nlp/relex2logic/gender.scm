(define gender
	(BindLink
		(VariableList
			(TypedVariableLink
				(VariableNode "$a-parse")
				(TypeNode "ParseNode")
			)
			(TypedVariableLink
				(VariableNode "$word")
				(TypeNode "WordInstanceNode")
			)	
			(TypedVariableLink
				(VariableNode "$gtype")
				(TypeNode "DefinedLinguisticConceptNode")
			)
		)
		(AndLink
			(WordInstanceLink
				(VariableNode "$word")
				(VariableNode "$a-parse")
			)
			(InheritanceLink 
			    (VariableNode "$word")
				(VariableNode "$gtype")
				
			)
			(InheritanceLink 
			    (VariableNode "$word")
				(DefinedLinguisticConceptNode "person")
				
			)
		)
   (ListLink
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-gender-rule")
			(ListLink
				(VariableNode "$word")
				(VariableNode "$gtype")
			)
		)
   )
))

(define (pre-gender-rule word gtype)
 (cond  ((or (string=? (cog-name gtype) "masculine") (string=? (cog-name gtype) "feminine"))	 
 (ListLink
	(gender-rule (cog-name (word-inst-get-lemma word)) (cog-name word) 
	(cog-name gtype))
 ))))
