; This rule processes a relatively new relex relation -- prepositional-adjectival --
; as in "the man in the sombrero" or "the moose under the table"
; (AN June 2015)


(define prepadj
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$noun")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$adj")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$noun")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$adj")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_prepadj")
                (ListLink
                    (VariableNode "$noun")
                    (VariableNode "$adj")
                )
            )
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
