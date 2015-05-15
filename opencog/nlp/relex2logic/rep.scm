(define rep
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
        (ImplicationLink
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
                    (DefinedLinguisticRelationshipNode "_rep")
                    (ListLink
                        (VariableNode "$pred")
			(VariableNode "$comp")     
                    )
                )
            )
            (ExecutionOutputLink
           	   (GroundedSchemaNode "scm: pre-rep-rule")
           	      (ListLink
           	        (VariableNode "$comp")  
			(VariableNode "$pred")         	         
                )
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "rep-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "rep-Rule") comp)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-rep-rule comp pred)
    	(rep-rule
		(word-inst-get-word-str comp) (cog-name comp)
		(word-inst-get-word-str pred) (cog-name pred)		              
	)
)


