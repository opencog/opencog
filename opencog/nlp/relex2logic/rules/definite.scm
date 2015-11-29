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
                (GroundedSchemaNode "scm: pre-definite-rule")
                (ListLink
                    (VariableNode "$lemma")
                    (VariableNode "$noun"))
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the
; existing r2l pipeline.  Huh ??? How can it not be needed?  It is
; used right up above!
;
; Here's what this does: given as input
;    (WordInstanceNode "ballgame@8cb61431")
;
; It generates the following output:
;    (InheritanceLink
;        (ConceptNode "ballgame@8cb61431")
;        (ConceptNode "ballgame"))
;     (ReferenceLink
;        (ConceptNode "ballgame@8cb61431")
;        (WordInstanceNode "ballgame@8cb61431"))
;     (EvaluationLink
;        (DefinedLinguisticPredicateNode "definite")
;        (ListLink (ConceptNode "ballgame@8cb61431")))
;
; Which strikes me as ugly, but wtf...


(define (pre-definite-rule lemma noun)
    ; XXX FIXME doe we really need the ListLink here ???
    (ListLink
        (definite-rule (cog-name lemma) (cog-name noun))
    )
)
