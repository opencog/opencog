; =====================================================================
; OrRule
;
; A<TV1>
; B<TV2>
; |-
; OrLink <TV>
;    A
;    B
;----------------------------------------------------------------------


(define pln-rule-or
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B"))
   (AndLink
    (VariableNode "$A")
    (VariableNode "$B"))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: pln-formula-or")
    (ListLink
     (VariableNode "$A")
     (VariableNode "$B")))))

(define (pln-formula-or A B)
  (cog-set-tv!
   (OrLink A B)
   (pln-formula-or-side-effect-free A B))
)

(define (pln-formula-or-side-effect-free A B)
  (let 
      ((sA (cog-stv-strength A))
       (sB (cog-stv-strength B))
       (cA (cog-stv-confidence A))
       (cB (cog-stv-confidence B)))
    (stv (- (+ sA sB) (* sA sB)) (min cA cB))))

; Name the rule
(define pln-rule-or-name (Node "pln-rule-or"))
(DefineLink pln-rule-or-name pln-rule-or)
