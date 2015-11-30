; check the atoms that should be grounded
;
; XXX FIXME replace the GroundedPredicates by
; (OrLink
;    (EqualLink
;      (VariableNode "$gtype")
;      (DefinedLinguisticRelationshipNode "masculine"))
;   (EqualLink
;      (VariableNode "$gtype")
;      (DefinedLinguisticRelationshipNode "feminine"))
;
(define (check-gender gtype)
    (if (or (string=? (cog-name gtype) "masculine") (string=? (cog-name gtype) "feminine"))
        (begin (stv 1 1))
        (begin (stv 0 1))
    )
)

(define gender
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$word")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$gtype")
                (TypeNode "DefinedLinguisticConceptNode")
            )
            (TypedVariableLink
                (VariableNode "$lemma")
                (TypeNode "WordNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$word")
                (VariableNode "$a-parse")
            )
            (InheritanceLink
                (VariableNode "$word")
                (VariableNode "$gtype")
            )
            (InheritanceLink
                (VariableNode "$word")
                (DefinedLinguisticConceptNode "person")
            )
            (LemmaLink
                (VariableNode "$word")
                (VariableNode "$lemma")
            )
            (EvaluationLink
                (GroundedPredicateNode "scm: check-gender")
                (ListLink
                    (VariableNode "$gtype")
                )
            )
        )
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: gender-rule")
            (ListLink
                (VariableNode "$lemma")
                (VariableNode "$word")
                (VariableNode "$gtype")
            )
        )
    )
)
