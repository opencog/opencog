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
        (EvaluationLink
                (GroundedPredicateNode "scm: check-gender")
                (ListLink
                    (VariableNode "$gtype")
                )
            )
        )
        (ListLink
            (ExecutionOutputLink
                (GroundedSchemaNode "scm: pre-gender-rule")
                (ListLink
                    (VariableNode "$word")
                    (VariableNode "$gtype")
                )
            )
        )
    )
)


(define (pre-gender-rule word gtype)
  (ListLink
        (gender-rule
            (cog-name (word-inst-get-lemma word)) (cog-name word)
            (cog-name gtype)
        )
  )
)
