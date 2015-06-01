(define compmod
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
                (DefinedLinguisticRelationshipNode "_compmod")
                (ListLink
                    (VariableNode "$pred")
		(VariableNode "$comp")     
                )
            )
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-compmod-rule")
       	      (ListLink
       	         (VariableNode "$pred")
		(VariableNode "$comp")           	         
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "compmod-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "compmod-Rule") comp)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-compmod-rule pred comp)
    	(compmod-rule (word-inst-get-word-str pred) (cog-name pred)
		(word-inst-get-word-str comp) (cog-name comp)              
	)
)


