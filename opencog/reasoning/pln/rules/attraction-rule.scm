; =============================================================================
; AttractionRule
;
; AndLink
;   SubsetLink
;       A
;       B
;   SubsetLink
;       NotLink
;           A
;       B
; |-
; AttractionLink
;   A
;   B
;
; -----------------------------------------------------------------------------

(define pln-rule-attraction
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (SubsetLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (SubsetLink
                (NotLink
                    (VariableNode "$A"))
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-attraction")
            (ListLink
                (AttractionLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (SubsetLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (SubsetLink
                    (NotLink
                        (VariableNode "$A"))
                    (VariableNode "$B"))))))

(define (pln-formula-attraction AB SAB SnAB)
    (cog-set-tv!
        AB
        (pln-formula-attraction-side-effect-free AB SAB SnAB)
    )
)

(define (pln-formula-attraction-side-effect-free AB SAB SnAB)
    (let
        (
            (sSAB (cog-stv-strength SAB))
            (cSAB (cog-stv-confidence SAB))
            (sSnAB (cog-stv-strength SnAB))
            (cSnAB (cog-stv-confidence SnAB))
    
        )
        (if 
            (<= sSAB sSnAB)
            (stv 0 (min cSAB cSnAB))
            (stv (- sSAB sSnAB) (min cSAB cSnAB)))))
