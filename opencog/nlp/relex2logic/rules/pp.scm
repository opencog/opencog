; This rule assigns the relation between a preposition and its object-noun,
; such as "on the table" and "in bed."
; (AN June 2015)


(define pp
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$noun" "WordInstanceNode")
			(var-decl "$prep" "WordInstanceNode")
        )
        (AndLink
			(word-in-parse "$noun" "$a-parse")
			(word-in-parse "$prep" "$a-parse")
			(dependency "_pobj" "$prep" "$noun")
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-pp-rule")
       	      (ListLink
       	         (VariableNode "$prep")
       	         (VariableNode "$noun")
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-pp-rule prep noun)
  (pp-rule (cog-name (word-inst-get-lemma prep)) (cog-name prep)
	        (cog-name (word-inst-get-lemma noun)) (cog-name noun)
  )
)
