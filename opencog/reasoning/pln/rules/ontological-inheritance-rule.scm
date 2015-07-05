; =============================================================================
; OntologicalInheritanceRule
;
; AndLink
;   InheritanceLink
;       A
;       B
;   InheritanceLink
;       B
;       A
; |-
; OntologicalInheritanceLink
;   A
;   B
;
; -----------------------------------------------------------------------------

(define pln-rule-ontological-inheritance
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (InheritanceLink
                (VariableNode "$B")
                (VariableNode "$A")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-ontological-inheritance")
            (ListLink
                (OntologicalInheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (VariableNode "$B")
                    (VariableNode "$A"))))))

(define (pln-formula-ontological-inheritance OAB AB BA)
    (cog-set-tv!
        OAB
        (pln-formula-ontological-inheritance-side-effect-free OAB AB BA)
    )
)

(define (pln-formula-ontological-inheritance-side-effect-free OAB AB BA)
    (let
        ((sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (sBA (cog-stv-strength BA))
         (cBA (cog-stv-confidence BA))
        )
        (if 
            (<= sAB sBA)
            (stv 0 (min cAB cBA))
            (stv (- sAB sBA) (min cAB cBA)))))

