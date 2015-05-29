(define comp
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
                (DefinedLinguisticRelationshipNode "_comp")
                (ListLink
                    (VariableNode "$comp")
                    (VariableNode "$pred")
                )
            )
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-comp-rule")
       	      (ListLink
       	         (VariableNode "$comp")
       	         (VariableNode "$pred")
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "comp-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "comp-Rule") comp)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-comp-rule comp pred)
    	(complement-rule (word-inst-get-word-str comp) (cog-name comp)
	(word-inst-get-word-str pred) (cog-name pred)
              
    )
)


