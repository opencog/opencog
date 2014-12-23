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

; Test AndLink rule. For now the TV formula assumes the 2 concepts are
; flat, or constant, in other words it's just fuzzy values, we ignore
; the confidence for now.

(define pln-rule-and
  (BindLink
   (ListLink
    (VariableNode "$A")
    (VariableNode "$B")
    (TypedVariableLink
     (VariableNode "$A")
     (VariableTypeNode "PredicateNode"))
    (TypedVariableLink
     (VariableNode "$B")
     (VariableTypeNode "PredicateNode")))
   (ImplicationLink
    (AndLink
     (VariableNode "$A")
     (VariableNode "$B"))
    (ExecutionLink
     (GroundedSchemaNode "scm: pln-formula-and")
     (ListLink
      (VariableNode "$A")
      (VariableNode "$B"))))))

(define (pln-formula-and A B)
  (cog-set-tv!
   (AndLink A B)
   (pln-formula-and-side-effect-free A B))
)

(define (pln-formula-and-side-effect-free A B)
  (let 
      ((sA (cog-stv-strength A))
       (sB (cog-stv-strength B)))
    (stv (min sA sB) 1)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Some test data (to be removed afterwards) ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(PredicateNode "A" (stv 0.2 1))

(PredicateNode "B" (stv 0.6 1))
