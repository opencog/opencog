; =============================================================================
; Term Probability Rule
; 
; AndLink
;   LinkType
;       A
;       B
;   LinkType
;       B
;       A
; A
; |-
; B
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-term-probability
    (BindLink
        (VariableList
            (VariableNode "$A")
            (TypedVariableLink
                (VariableNode "$D")
                (TypeChoice
                    (TypeNode "InheritanceLink")
                    (TypeNode "ImplicationLink")
                    (TypeNode "SubsetLink")))
            (TypedVariableLink
                (VariableNode "$E")
                (TypeChoice
                    (TypeNode "InheritanceLink")
                    (TypeNode "ImplicationLink")
                    (TypeNode "SubsetLink"))))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$D")
            (VariableNode "$E"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-term-probability")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$D")
                (VariableNode "$E")))))

(define (pln-formula-term-probability A AB BA)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (sBA (cog-stv-strength BA))
         (cBA (cog-stv-confidence BA)))
        (cond
            [(and
                (= (gar AB) A)
                (= (gdr AB) (gar BA))
                (= (gdr BA) A)
                (not (= (gdr AB) (gar AB)))
                (= (cog-type AB) (cog-type BA)))
             (cog-set-tv! (gdr AB) (stv (/ (* sA sAB) sBA) (min (min cAB cBA) cA)))
            ]
         )
    )
)
