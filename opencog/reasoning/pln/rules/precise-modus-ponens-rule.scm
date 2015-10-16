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
;       precise-modus-ponens-inheritance-rule
;       precise-modus-ponens-implication-rule
;       precise-modus-ponens-subset-rule
;

; -----------------------------------------------------------------------------
(load "formulas.scm")

(define precise-modus-ponens-inheritance-rule
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
            (GroundedSchemaNode "scm: precise-modus-ponens-formula")
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

(define precise-modus-ponens-implication-rule
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
            (GroundedSchemaNode "scm: precise-modus-ponens-formula")
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

(define precise-modus-ponens-subset-rule
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
            (GroundedSchemaNode "scm: precise-modus-ponens-formula")
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

(define (precise-modus-ponens-formula A AB notAB B)
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
                (precise-modus-ponens-strength-formula sA sAB snotAB) 
                (min (min cAB cnotAB) cA)))))
