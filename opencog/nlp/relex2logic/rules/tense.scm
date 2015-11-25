;Both link grammar and relex output are selected because of the pattern 
;Example : "He gave me the book."
;(DefinedLinguisticConceptNode "past") and (DefinedLinguisticConceptNode ".v-d")
;I just want to avoid the Link Grammar output. There might be another way to do this
(define (check-tense tense)
    (if (string-contains (cog-name tense) ".")
        (begin (stv 0 1))
        (begin (stv 1 1))
    )
)

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
            (EvaluationLink
                (GroundedPredicateNode "scm: check-tense")
                (ListLink
                    (VariableNode "$tense")
                )
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