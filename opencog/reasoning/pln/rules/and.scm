; =====================================================================
; AndRule
; (http://wiki.opencog.org/w/AndRule TODO)
;
; A<TV1>
; B<TV2>
; |-
; AndLink <TV>
;    A
;    B
;----------------------------------------------------------------------


(define pln-rule-and
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B"))
   (AndLink
    (VariableNode "$A")
    (VariableNode "$B"))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: pln-formula-and")
    (ListLink
     (VariableNode "$A")
     (VariableNode "$B")))))

(define (pln-formula-and A B)
  (cog-set-tv!
   (AndLink A B)
   (pln-formula-and-side-effect-free A B))
)

(define (pln-formula-and-side-effect-free A B)
  (let 
      ((sA (cog-stv-strength A))
       (sB (cog-stv-strength B))
       (cA (cog-stv-confidence A))
       (cB (cog-stv-confidence B)))
    (stv (* sA sB) (min cA cB))))

; Name the rule
;(define pln-rule-and-name (Node "pln-rule-and"))
;(DefineLink pln-rule-and-name pln-rule-and)
