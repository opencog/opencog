; =============================================================================
; ModusPonensRule
; 
; AndLink
;   LinkType
;       A
;       B
;   A
; |-
;   B
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-modus-ponens
    (BindLink
        (VariableList
            (VariableNode "$A")
            (TypedVariableLink
                (VariableNode "$C")
                (TypeChoice
                    (TypeNode "InheritanceLink")
                    (TypeNode "ImplicationLink")
                    (TypeNode "SubsetLink"))))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$C"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-modus-ponens")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$C")))))

(define (pln-formula-modus-ponens A AB)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (snotAB 0.2)
         (cnotAB 1))
        (cond
            [(= (gar AB) A)
             (cog-set-tv! 
                (gdr AB) 
                (stv 
                    (precise-modus-ponens-formula sA sAB snotAB) 
                    (min (min cAB cnotAB) cA)))
            ]
         )
    )
)

