(define because
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$pred")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$comp")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$pred")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$comp")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_%because")
                (ListLink
                    (VariableNode "$pred")
					(VariableNode "$comp")     
                )
            )
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-because-rule")
       	      (ListLink
       	        (VariableNode "$comp")  
				(VariableNode "$pred")         	         
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "because-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "because-Rule") comp)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-because-rule comp pred)
    	(because-rule
		(word-inst-get-word-str comp) (cog-name comp)
		(word-inst-get-word-str pred) (cog-name pred)		              
	)
)


