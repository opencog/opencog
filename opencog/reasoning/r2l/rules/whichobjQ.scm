(define whichobjQ
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$subj")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$verb")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$obj")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$qVar")
                (TypeNode "WordInstanceNode")
            )		
        )
        (ImplicationLink
            (AndLink
                (WordInstanceLink
                    (VariableNode "$subj")
                    (VariableNode "$a-parse")
                )
                (WordInstanceLink
                    (VariableNode "$verb")
                    (VariableNode "$a-parse")
                )
                (WordInstanceLink
                    (VariableNode "$obj")
                    (VariableNode "$a-parse")
                )
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_subj")
                    (ListLink
                        (VariableNode "$verb")
                        (VariableNode "$subj")
                    )
                )
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_obj")
                    (ListLink
                        (VariableNode "$verb")
                        (VariableNode "$obj")
                    )
                )
		(EvaluationLink
   			(DefinedLinguisticRelationshipNode "_det")
  			 (ListLink
     				(VariableNode "$obj")
      				(VariableNode "$qVar")
			)
		)
		(InheritanceLink
			(VariableNode "$qVar")
			(DefinedLinguisticConceptNode "which")
		)
            )
            (ExecutionOutputLink
           	   (GroundedSchemaNode "scm: pre-whichobjQ-rule")
           	      (ListLink
           	         (VariableNode "$subj")
           	         (VariableNode "$verb")
           	         (VariableNode "$obj")
                )
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "whichobjQ-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "whichobjQ-Rule") whichobjQ)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-whichobjQ-rule subj verb obj)
    (whichobjQ-rule (word-inst-get-word-str obj) (cog-name obj)
              (word-inst-get-word-str verb) (cog-name verb)
              (word-inst-get-word-str subj) (cog-name subj)
    )
)


