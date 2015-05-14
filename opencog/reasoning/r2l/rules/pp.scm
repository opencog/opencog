(define pp
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$noun")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$prep")
                (TypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
            (AndLink
                (WordInstanceLink
                    (VariableNode "$noun")
                    (VariableNode "$a-parse")
                )
                (WordInstanceLink
                    (VariableNode "$prep")
                    (VariableNode "$a-parse")
                )
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_pobj")
                    (ListLink
                        (VariableNode "$prep")
                        (VariableNode "$noun")
                    )
                )
            )
            (ExecutionOutputLink
           	   (GroundedSchemaNode "scm: pre-pp-rule")
           	      (ListLink
           	         (VariableNode "$prep")
           	         (VariableNode "$noun")
                )
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "pp-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "pp-Rule") pp)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-pp-rule prep noun)
    	(pp-rule (word-inst-get-word-str prep) (cog-name prep)
	(word-inst-get-word-str noun) (cog-name noun)
              
    )
)


