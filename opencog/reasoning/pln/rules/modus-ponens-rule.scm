; =============================================================================
; ModusPonensRule
; 
; AndLink
;   LinkType
;       A
;       B
;   A
; |-
;   B
; Due to pattern matching issues, currently the file has been divided into 3 
; parts, each pertaining to different links. The rules are :-
;       modus-ponens-inheritance-rule
;       modus-ponens-implication-rule
;       modus-ponens-subset-rule
;
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define modus-ponens-inheritance-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: modus-ponens-formula")
            (ListLink
                (VariableNode "$A")
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define modus-ponens-implication-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (ImplicationLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: modus-ponens-formula")
            (ListLink
                (VariableNode "$A")
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define modus-ponens-subset-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (SubsetLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: modus-ponens-formula")
            (ListLink
                (VariableNode "$A")
                (SubsetLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$B")))))

(define (modus-ponens-formula A AB B)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (snotAB 0.2)
         (cnotAB 1))
        (cog-set-tv!
            B
            (stv 
                (precise-modus-ponens-strength-formula sA sAB snotAB) 
                (min (min cAB cnotAB) cA)))))

