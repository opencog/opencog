; This rule handles the _%atTime relation, whose coverage is a littel unpredictable.  It will catch
; most when-phrases and questions; beyond that you should check the relex output to see whether it's getting
; assigned or not.
; (AN June 2015)

(define atTime
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
                (DefinedLinguisticRelationshipNode "_%atTime")
                (ListLink
                    (VariableNode "$pred")
		    (VariableNode "$comp")
                )
            )
        )
       (ListLink
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-atTime-rule")
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
(define (pre-atTime-rule comp pred)
  (ListLink
    (attime-rule
		(cog-name (word-inst-get-lemma comp)) (cog-name comp)
		(cog-name (word-inst-get-lemma pred)) (cog-name pred)
	)
 )
)
