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
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$noun")
                (VariableNode "$a-parse")
            )
            (InheritanceLink
                (VariableNode "$noun")
                (DefinedLinguisticConceptNode "definite")
            )
        )
        (ListLink
            (ExecutionOutputLink
                (GroundedSchemaNode "scm: pre-definite-rule")
                (ListLink (VariableNode "$noun"))
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the
; existing r2l pipeline.  Huh ??? How can it not be needed?  It is
; used right up above!
(define (pre-definite-rule noun)
  (ListLink
    	(definite-rule (cog-name (word-inst-get-lemma noun)) (cog-name noun)
    )
  )
)
