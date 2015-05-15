(define det
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
                (VariableNode "$det")
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
                    (VariableNode "$det")
                    (VariableNode "$a-parse")
                )
                (InheritanceLink
   			(VariableNode "$noun")
   			(DefinedLinguisticConceptNode "definite")
		)
		(EvaluationLink
			(DefinedLinguisticRelationshipNode "_det")
			(ListLink
				(VariableNode "$noun")
				(VariableNode "$det")
			)
		)
            )
            (ExecutionOutputLink
          	  (GroundedSchemaNode "scm: pre-det-rule")
           	      (ListLink
           	         (VariableNode "$noun")
           	         (VariableNode "$det")
			
		)
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "det-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "det-Rule") det)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-det-rule noun det)
    (det-rule (word-inst-get-word-str noun)(cog-name noun)
		(word-inst-get-word-str det)(cog-name det)
    )
)


