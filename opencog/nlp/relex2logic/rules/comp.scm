; This rule constructs sentential complements that have complementizers.
; e.g. in "I know that you are a fool." this constructs the relation between "that" and "are"
; (AN June 2015)

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
       (ListLink
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-comp-rule")
       	      (ListLink
       	         (VariableNode "$comp")
       	         (VariableNode "$pred")
            )
        )
      )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-comp-rule comp pred)
 (ListLink
  (complement-rule (cog-name (word-inst-get-lemma comp)) (cog-name comp)
	(cog-name (word-inst-get-lemma pred)) (cog-name pred)
  )
 )
)
