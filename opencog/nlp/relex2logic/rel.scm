(define rel
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
                (VariableNode "$rel")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$pred")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$rel")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_rel")
                (ListLink
                    (VariableNode "$rel")
                    (VariableNode "$pred")
                )
            )
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-rel-rule")
       	      (ListLink
       	         (VariableNode "$rel")
       	         (VariableNode "$pred")
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "rel-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "rel-Rule") rel)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-rel-rule rel pred)
    	(complement-rule (word-inst-get-word-str rel) (cog-name rel)
	(word-inst-get-word-str pred) (cog-name pred)
              
    )
)


