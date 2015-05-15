; NB: this rule could not be assimilated to the amod rule like the adnounials were assimilated to the advmod rule because
; these phrases are picked out in Relex for further processing in phrases like "the man to whom I spoke."

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
                (VariableNode "$adv")
                (TypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
            (AndLink
                (WordInstanceLink
                    (VariableNode "$noun")
                    (VariableNode "$a-parse")
                )
                (WordInstanceLink
                    (VariableNode "$adv")
                    (VariableNode "$a-parse")
                )
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_prepadj")
                    (ListLink
                        (VariableNode "$noun")
                        (VariableNode "$adv")
                    )
                )
            )
            (ExecutionOutputLink
           	   (GroundedSchemaNode "scm: pre-prepadj-rule")
           	      (ListLink
           	         (VariableNode "$noun")
           	         (VariableNode "$adv")
                )
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "prepadj-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "prepadj-Rule") prepadj)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-prepadj-rule noun adv)
    (prepadj-rule (word-inst-get-word-str noun) (cog-name noun)
              (word-inst-get-word-str adv) (cog-name adv)
    )
)


