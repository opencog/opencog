; this rule handles the _%because relation, which mainly handles because-phrases and
; why-questions.
; (AN June 2015)

(define because
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
                (DefinedLinguisticRelationshipNode "_%because")
                (ListLink
                    (VariableNode "$pred")
					(VariableNode "$comp")
                )
            )
        )
       (ListLink
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-because-rule")
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
(define (pre-because-rule comp pred)
 (ListLink
  (because-rule
		(cog-name (word-inst-get-lemma comp)) (cog-name comp)
		(cog-name (word-inst-get-lemma pred)) (cog-name pred)
	)
 )
)
