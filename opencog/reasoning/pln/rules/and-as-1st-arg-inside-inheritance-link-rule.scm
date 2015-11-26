; =============================================================================
; And as 1st arg inside inheritance link rule
;
; AndLink
;   InheritanceLink
;       A
;       C
;   InheritanceLink
;       B
;       C
; |-
; InheritanceLink
;   AndLink
;       A
;       B
;   C
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-and-as-1st-arg-inside-inheritance-link
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
                (VariableNode "$C"))
            (InheritanceLink
                (VariableNode "$B")
                (VariableNode "$C"))
            (NotLink
                (EqualLink
                    (VariableNode "$A")
                    (VariableNode "$B"))))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-and-as-1st-arg")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$C"))
                (InheritanceLink
                    (VariableNode "$B")
                    (VariableNode "$C"))
                (AndLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (AndLink
                        (VariableNode "$A")
                        (VariableNode "$B"))
                    (VariableNode "$C"))))))

(define (pln-formula-and-as-1st-arg A B C AC BC AB ABC)
    (
        (cog-set-tv! 
            AB 
            (stv 
                (* (cog-stv-strength A) (cog-stv-strength B))
                (min (cog-stv-confidence A) (cog-stv-confidence B))))
        (cog-set-tv!
            ABC
            (stv
                (/
                    (* 
                        (* (cog-stv-strength AC) (cog-stv-strength BC))
                        (* (cog-stv-strength A) (cog-stv-strength B)))
                    (* (cog-stv-strength C) (cog-stv-strength AB)))
                (min 
                    (cog-stv-confidence A)
                    (cog-stv-confidence B)
                    (cog-stv-confidence C)
                    (cog-stv-confidence AC)
                    (cog-stv-confidence BC))))))

