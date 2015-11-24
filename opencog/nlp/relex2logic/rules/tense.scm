(define tense
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$verb")
                (TypeNode "WordInstanceNode")
            )
		    (TypedVariableLink
                (VariableNode "$tense")
                (TypeNode "DefinedLinguisticConceptNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$verb")
                (VariableNode "$a-parse")
            )
            (PartOfSpeechLink
                (VariableNode "$verb")
                (DefinedLinguisticConceptNode "verb")
            )
            (InheritanceLink
                (VariableNode "$verb")
                (VariableNode "$tense")
            )
        )
        (ListLink
            (ExecutionOutputLink
       	        (GroundedSchemaNode "scm: pre-tense-rule")
       	        (ListLink
                    (VariableNode "$verb")
                    (VariableNode "$tense")
                )
            )
        )
    )
)

(define (pre-tense-rule verb tense)
  (ListLink
    (tense-rule (cog-name (word-inst-get-lemma  verb)) (cog-name verb) (cog-name tense))
  )
)