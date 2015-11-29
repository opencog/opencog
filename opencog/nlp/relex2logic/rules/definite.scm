; This rule simply inherits the linguistic concept of being definite to
; any definite noun such as "that man."
; (AN June 2015)

(define definite
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
                (VariableNode "$lemma")
                (TypeNode "WordNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$noun")
                (VariableNode "$a-parse")
            )
            (LemmaLink
                (VariableNode "$noun")
                (VariableNode "$lemma")
            )
            (InheritanceLink
                (VariableNode "$noun")
                (DefinedLinguisticConceptNode "definite")
            )
        )
        (ListLink
            (ExecutionOutputLink
                (GroundedSchemaNode "scm: definite-rule")
                (ListLink
                    (VariableNode "$lemma")
                    (VariableNode "$noun"))
            )
        )
    )
)
