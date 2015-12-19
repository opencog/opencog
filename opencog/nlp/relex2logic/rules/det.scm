
(define det
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$noun" "WordInstanceNode")
			(var-decl "$det" "WordInstanceNode")
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$noun")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$det")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_det")
                (ListLink
                    (VariableNode "$noun")
                    (VariableNode "$det")
                )
            )
            (InheritanceLink
                (VariableNode "$noun")
                (DefinedLinguisticConceptNode "definite")
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


(define (pre-det-rule noun det)
    (det-rule  (cog-name (word-inst-get-lemma  noun)) (cog-name noun)
              (choose-var-name) (cog-name (word-inst-get-lemma det))
    )
)
