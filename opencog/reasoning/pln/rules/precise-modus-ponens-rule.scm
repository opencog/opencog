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
; Due to pattern matching issues, currently the file has been divided into 3 
; parts, each pertaining to different links. The rules are :-
;       pln-rule-precise-modus-ponens-inheritance
;       pln-rule-precise-modus-ponens-implication
;       pln-rule-precise-modus-ponens-subset
;

; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-precise-modus-ponens-inheritance
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (InheritanceLink
                (NotLink
                    (VariableNode "$A"))
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-precise-modus-ponens")
            (ListLink
                (VariableNode "$A")
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (NotLink
                        (VariableNode "$A"))
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define pln-rule-precise-modus-ponens-implication
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (ImplicationLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (ImplicationLink
                (NotLink
                    (VariableNode "$A"))
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-precise-modus-ponens")
            (ListLink
                (VariableNode "$A")
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (ImplicationLink
                    (NotLink
                        (VariableNode "$A"))
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define pln-rule-precise-modus-ponens-subset
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (SubsetLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (SubsetLink
                (NotLink
                    (VariableNode "$A"))
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-precise-modus-ponens")
            (ListLink
                (VariableNode "$A")
                (SubsetLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (SubsetLink
                    (NotLink
                        (VariableNode "$A"))
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define (pln-formula-precise-modus-ponens A AB notAB B)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (snotAB (cog-stv-strength notAB))
         (cnotAB (cog-stv-confidence notAB)))
        (cog-set-tv!
            B
            (stv 
                (precise-modus-ponens-formula sA sAB snotAB) 
                (min (min cAB cnotAB) cA)))))
