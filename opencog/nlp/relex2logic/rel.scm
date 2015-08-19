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
       (ListLink
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-rel-rule")
       	      (ListLink
       	         (VariableNode "$rel")
       	         (VariableNode "$pred")
            )
        )
      )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-rel-rule rel pred)
 (ListLink
  (complement-rule (word-inst-get-word-str rel) (cog-name rel)
	(word-inst-get-word-str pred) (cog-name pred)

  )
 )
)
