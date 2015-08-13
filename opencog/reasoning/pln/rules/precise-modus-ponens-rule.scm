; =============================================================================
; PreciseModusPonensRule
; 
; AndLink
;   LinkType
;       A
;       B
;   LinkType
;       NotLink
;           A
;       B
;   A
; |-
;   B
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-precise-modus-ponens
    (BindLink
        (VariableList
            (VariableNode "$A")
            (TypedVariableLink
                (VariableNode "$C")
                (TypeChoice
                    (TypeNode "InheritanceLink")
                    (TypeNode "ImplicationLink")
                    (TypeNode "SubsetLink")))
            (TypedVariableLink
                (VariableNode "$D")
                (TypeChoice
                    (TypeNode "InheritanceLink")
                    (TypeNode "ImplicationLink")
                    (TypeNode "SubsetLink"))))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$C")
            (VariableNode "$D"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-precise-modus-ponens")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$C")
                (VariableNode "$D")))))

(define (pln-formula-precise-modus-ponens A AB notAB)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (snotAB (cog-stv-strength notAB))
         (cnotAB (cog-stv-confidence notAB)))
        (cond
            [(and
                (= (gar AB) A)
                (= (gar (gar notAB)) A)
                (= (gdr notAB) (gdr AB))
                (not (= (gar AB) (gdr AB)))
                (= (cog-type (gar notAB)) 'NotLink)
                (= (cog-type AB) (cog-type notAB)))
             (cog-set-tv! (gdr AB) (stv (precise-modus-ponens-formula sA sAB snotAB) (min (min cAB cnotAB) cA)))
            ]
         )
    )
)
