;"He stood at the goal line."
(define nn
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$N1" "WordInstanceNode")
			(var-decl "$N2" "WordInstanceNode")
        )
        (AndLink
			(word-in-parse "$N1" "$a-parse")
			(word-in-parse "$N2" "$a-parse")
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_nn")
                (ListLink
                    (VariableNode "$N1")
                    (VariableNode "$N2")
                )
            )
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-nn-rule")
       	      (ListLink
       	         (VariableNode "$N1")
       	         (VariableNode "$N2")
            )
        )
    )
)


(define (pre-nn-rule N1 N2)
    (nn-rule (cog-name (word-inst-get-lemma  N1)) (cog-name N1)
             (cog-name (word-inst-get-lemma N2)) (cog-name N2)
    )
)
