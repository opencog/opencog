; =====================================================================
; NotRule
; (http://wiki.opencog.org/w/NotRule TODO)
;
; A<TV1>
; |-
; NotLink <TV>
;    A
;----------------------------------------------------------------------

; Test NotLink rule. For now the TV formula assumes the concept is
; flat, or constant, in other words it's just fuzzy values, we ignore
; the confidence for now.

(define pln-rule-not
  (BindLink
    (ListLink
      (VariableNode "$A")
      (TypedVariableLink
       (VariableNode "$A")
       (VariableTypeNode "PredicateNode")))
    (ImplicationLink
      (VariableNode "$A")
      (ExecutionOutputLink
        (GroundedSchemaNode "scm: pln-formula-not")
        (ListLink
          (VariableNode "$A"))))))

(define (pln-formula-not A)
  (cog-set-tv!
   (NotLink A)
   (pln-formula-not-side-effect-free A))
)

(define (pln-formula-not-side-effect-free A)
  (let 
      ((sA (cog-stv-strength A)))
    (stv (- SA 1) 1)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Some test data (to be removed afterwards) ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(PredicateNode "A" (stv 0.2 1))
