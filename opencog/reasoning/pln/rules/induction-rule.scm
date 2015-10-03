; =============================================================================
; InductionRule
; 
; AndLink
;   LinkType
;       A
;       B
;   LinkType
;       A
;       C
; |-
; LinkType
;   B
;   C
;
; Due to pattern matching issues, currently the file has been divided into 3 
; parts, each pertaining to different links. The rules are :-
;       pln-rule-induction-inheritance
;       pln-rule-induction-implication
;       pln-rule-induction-subset
;

; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-induction-inheritance
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$C"))
            (NotLink
                (EqualLink
                    (VariableNode "$B")
                    (VariableNode "$C"))))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-induction")
            (ListLink
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$C"))
                (InheritanceLink
                    (VariableNode "$B")
                    (VariableNode "$C"))))))
    
(define pln-rule-induction-implication
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (ImplicationLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (ImplicationLink
                (VariableNode "$A")
                (VariableNode "$C"))
            (NotLink
                (EqualLink
                    (VariableNode "$B")
                    (VariableNode "$C"))))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-induction")
            (ListLink
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$C"))
                (ImplicationLink
                    (VariableNode "$B")
                    (VariableNode "$C"))))))

(define pln-rule-induction-subset
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (SubsetLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (SubsetLink
                (VariableNode "$A")
                (VariableNode "$C"))
            (NotLink
                (EqualLink
                    (VariableNode "$B")
                    (VariableNode "$C"))))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-induction")
            (ListLink
                (SubsetLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (SubsetLink
                    (VariableNode "$A")
                    (VariableNode "$C"))
                (SubsetLink
                    (VariableNode "$B")
                    (VariableNode "$C"))))))

(define (pln-formula-induction AB AC BC)
    (define A (gar AB))
    (define B (gdr AB))
    (define C (gdr AC))
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sB (cog-stv-strength B))
         (cB (cog-stv-confidence B))
         (sC (cog-stv-strength C))
         (cC (cog-stv-confidence C))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (sAC (cog-stv-strength AC))
         (cAC (cog-stv-confidence AC)))
        (cog-set-tv!
            BC
            (stv 
                (simple-deduction-formula sB sA sC (inversion-formula sAB sA sB) sAC) 
                (min cAB cAC)))))
                
; =============================================================================

; Name the rules
(define pln-rule-induction-inheritance-name (Node "pln-rule-induction-inheritance"))
(DefineLink pln-rule-induction-inheritance-name pln-rule-induction-inheritance)

(define pln-rule-induction-implication-name (Node "pln-rule-induction-implication"))
(DefineLink pln-rule-induction-implication-name pln-rule-induction-implication)

(define pln-rule-induction-subset-name (Node "pln-rule-induction-subset"))
(DefineLink pln-rule-induction-subset-name pln-rule-induction-subset)
