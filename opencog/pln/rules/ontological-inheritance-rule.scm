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

(define ontological-inheritance-rule
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
            (GroundedSchemaNode "scm: ontological-inheritance-formula")
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

(define (ontological-inheritance-formula OAB AB BA)
    (cog-set-tv!
        OAB
        (ontological-inheritance-side-effect-free-formula OAB AB BA)
    )
)

(define (ontological-inheritance-side-effect-free-formula OAB AB BA)
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


; Name the rule
(define ontological-inheritance-rule-name
  (DefinedSchemaNode "ontological-inheritance-rule"))
(DefineLink
  ontological-inheritance-rule-name
  ontological-inheritance-rule)
