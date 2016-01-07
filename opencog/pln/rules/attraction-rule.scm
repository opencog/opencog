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

(define attraction-rule
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
            (GroundedSchemaNode "scm: attraction-formula")
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

(define (attraction-formula AB SAB SnAB)
    (cog-set-tv!
        AB
        (attraction-side-effect-free-formula AB SAB SnAB)
    )
)

(define (attraction-side-effect-free-formula AB SAB SnAB)
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

; Name the rule
(define attraction-rule-name (DefinedSchemaNode "attraction-rule"))
(DefineLink attraction-rule-name attraction-rule)
