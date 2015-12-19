; This rule processes a relatively new relex relation -- prepositional-adjectival --
; as in "the man in the sombrero" or "the moose under the table"
; (AN June 2015)


(define prepadj
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$noun" "WordInstanceNode")
			(var-decl "$adj" "WordInstanceNode")
        )
        (AndLink
			(word-in-parse "$noun" "$a-parse")
			(word-in-parse "$adj" "$a-parse")
			(dependency "_prepadj" "$noun" "$adj")
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-prepadj-rule")
       	      (ListLink
       	         (VariableNode "$noun")
       	         (VariableNode "$adj")
            )
        )
    )
)

;;ToDo: XXX FIXME Define prepadj-rule
; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-prepadj-rule noun adj)
    (prepadj-rule (cog-name (word-inst-get-lemma  noun)) (cog-name noun)
                  (cog-name (word-inst-get-lemma adj)) (cog-name adj)
    )
)
