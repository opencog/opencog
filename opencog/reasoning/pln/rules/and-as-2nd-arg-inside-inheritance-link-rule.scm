; =============================================================================
; And as 2nd arg inside inheritance link rule
;
; AndLink
;   InheritanceLink
;       A
;       B
;   InheritanceLink
;       A
;       C
; |-
; InheritanceLink
;   A
;   AndLink
;       B
;       C
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-and-as-2nd-arg-inside-inheritance-link
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C")
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$C"))
            (NotLink
                (EqualLink
                    (VariableNode "$A")
                    (VariableNode "$B"))))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-and-as-2nd-arg")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$C"))
                (AndLink
                    (VariableNode "$B")
                    (VariableNode "$C"))
                (InheritanceLink
                    (VariableNode "$A")
                    (AndLink
                        (VariableNode "$B")
                        (VariableNode "$C")))))))

(define (pln-formula-and-as-2nd-arg A B C AB AC BC ABC)
    (
        (cog-set-tv! 
            BC
            (stv 
                (* (cog-stv-strength B) (cog-stv-strength C))
                (min (cog-stv-confidence B) (cog-stv-confidence C))))
        (cog-set-tv!
            ABC
            (stv
                (/
                    (* 
                        (* (cog-stv-strength AB) (cog-stv-strength BC))
                        (* (cog-stv-strength A) (cog-stv-strength AC)))
                    (* (cog-stv-strength C) (cog-stv-strength B)))
                (min
                    (cog-stv-confidence A)
                    (cog-stv-confidence B)
                    (cog-stv-confidence C)
                    (cog-stv-confidence AB)
                    (cog-stv-confidence AC))))))
        
))))

