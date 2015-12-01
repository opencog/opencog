; This rule is for negation of predicates, as in "You do not smell good, sir."
; (AN June 2015)


(define neg
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
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$pred")
                (VariableNode "$a-parse")
            )
            (InheritanceLink
                (VariableNode "$pred")
                (DefinedLinguisticConceptNode "negative")
            )
        )
        (ExecutionOutputLink
           (GroundedSchemaNode "scm: pre-neg-rule")
              (ListLink
                 (VariableNode "$pred")
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-neg-rule pred)
   (ListLink
    (negative-rule (cog-name (word-inst-get-lemma pred)) (cog-name pred)   
    )
   )
)
