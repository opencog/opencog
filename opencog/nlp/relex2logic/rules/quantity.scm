; This is a new rule -- the previous version of relex had an all-rule,
; but no quantity rule.  I am not porting the all-rule, because it
; seems silly to pick out that one quantifier for its own rule, but
; none of the others -- some, many, few etc. etc. which also get _quantity
; (AN June 2015)

(define quantity
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$noun" "WordInstanceNode")
			(var-decl "$quant" "WordInstanceNode")
        )
        (AndLink
			(word-in-parse "$noun" "$a-parse")
			(word-in-parse "$quant" "$a-parse")
			(dependency "_quantity" "$noun" "$quant")
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-quantity-rule")
       	      (ListLink
       	         (VariableNode "$noun")
       	         (VariableNode "$quant")
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-quantity-rule noun quant)
    (quantity-rule (cog-name (word-inst-get-lemma noun)) (cog-name noun)
                   (cog-name (word-inst-get-lemma quant)) (cog-name quant)
    )
)
