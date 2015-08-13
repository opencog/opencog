; =============================================================================
; InversionRule
;
; LinkType
;   A
;   B
; |-
; LinkType
;   B
;   A
;
; -----------------------------------------------------------------------------

(define pln-rule-inversion
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (TypedVariableLink
                (VariableNode "$C")
                (TypeChoice
                    (TypeNode "InheritanceLink")
                    (TypeNode "ImplicationLink")
                    (TypeNode "SubsetLink"))))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-inversion")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")))))

(define (pln-formula-inversion A B AB)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sB (cog-stv-strength B))
         (cB (cog-stv-confidence B))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB)))
        (cond
            [(and
                (= (gar AB) A)
                (= (gdr AB) B))
             ((cog-type AB) (stv (/ (* sAB sB) (floor sA)) (min sA sB sAB))
                B A)
            ]
         )
    )
)
