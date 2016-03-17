; This rule handles the _%atTime relation, whose coverage is a littel unpredictable.  It will catch
; most when-phrases and questions; beyond that you should check the relex output to see whether it's getting
; assigned or not.
; (AN June 2015)

(define atTime
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$pred" "WordInstanceNode")
			(var-decl "$comp" "WordInstanceNode")
        )
        (AndLink
			(word-in-parse "$pred" "$a-parse")
			(word-in-parse "$comp" "$a-parse")
			(dependency "_%atTime" "$pred" "$comp")
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-atTime-rule")
       	      (ListLink
       	        (VariableNode "$comp")
		            (VariableNode "$pred")
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-atTime-rule comp pred)
    (attime-rule
		(cog-name (word-inst-get-lemma comp)) (cog-name comp)
		(cog-name (word-inst-get-lemma pred)) (cog-name pred)
	)
)
