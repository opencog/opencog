;
; Both link grammar and relex output are selected because of the
; pattern
; Example : "He gave me the book."
;   (DefinedLinguisticConceptNode "past") and
;   (DefinedLinguisticConceptNode ".v-d")
; We don't want to the Link Grammar subscript.
; A better way to do this would be to use TenseLink and SubscriptLink...
;
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
            (TypedVariableLink
                (VariableNode "$lemma")
                (TypeNode "WordNode")
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
            (LemmaLink
                (VariableNode "$verb")
                (VariableNode "$lemma")
            )
            (EvaluationLink
                (GroundedPredicateNode "scm: check-tense")
                (ListLink
                    (VariableNode "$tense")
                )
            )
        )
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: tense-rule")
            (ListLink
                (VariableNode "$lemma")
                (VariableNode "$verb")
                (VariableNode "$tense")
            )
        )
    )
)
