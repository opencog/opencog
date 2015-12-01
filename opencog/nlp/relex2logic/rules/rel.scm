; This rule relates relative-pronouns to the verbs in the relative clauses,
; as in "What happened to the pizza that I put in the fridge?" This rule relates "that" to "put"
; (AN June 2015)


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

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-rel-rule rel pred)
  (complement-rule (cog-name (word-inst-get-lemma  rel)) (cog-name rel)
                  (cog-name (word-inst-get-lemma  pred)) (cog-name pred)
  )
)
