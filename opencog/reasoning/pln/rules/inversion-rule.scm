; =============================================================================
; InversionRule
;
; LinkType
;   A
;   B
; |-
; LinkType
;   B
;   A
;
; Due to pattern matching issues, currently the file has been divided into 3 
; parts, each pertaining to different links. The rules are :-
;       pln-rule-inversion-inheritance
;       pln-rule-inversion-implication
;       pln-rule-inversion-subset
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-inversion-inheritance
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$B")
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-rule-inversion")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (VariableNode "$B")
                    (VariableNode "$A"))))))

(define pln-rule-inversion-implication
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$B")
            (ImplicationLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-rule-inversion")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (ImplicationLink
                    (VariableNode "$B")
                    (VariableNode "$A"))))))

(define pln-rule-inversion-subset
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$B")
            (SubsetLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-rule-inversion")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (SubsetLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (SubsetLink
                    (VariableNode "$B")
                    (VariableNode "$A"))))))

(define (pln-formula-inversion A B AB BA)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sB (cog-stv-strength B))
         (cB (cog-stv-confidence B))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB)))
        (cog-set-tv!
            BA
            (stv 
                (/ (* sAB sB) (floor sA)) 
                (min sA sB sAB)))))
