; =====================================================================
; NotConstructionRule
; (http://wiki.opencog.org/w/NotConstructionRule TODO)
;
; A<TV1>
; |-
; NotLink <TV>
;    A
;----------------------------------------------------------------------

(define pln-rule-not-construction
  (BindLink
     (VariableList
        (TypedVariableLink
           (VariableNode "$A")
           (TypeChoice
              (TypeNode "PredicateNode")
              (TypeNode "ConceptNode"))))
     (VariableNode "$A")
     (ExecutionOutputLink
        (GroundedSchemaNode "scm: pln-formula-not-construction")
        (ListLink
           (VariableNode "$A")))))

(define (pln-formula-not-construction A)
  (cog-set-tv!
   (NotLink A)
   (pln-formula-not-construction-side-effect-free A))
)

(define (negate x)
  (- 1 x))

(define (pln-formula-not-construction-side-effect-free A)
  (let ((sA (cog-stv-strength A))
        (cA (cog-stv-confidence A)))
    (stv (negate sA) cA)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Some test data (to be removed afterwards) ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Name the rule
(define pln-rule-not-construction-name
  (Node "pln-rule-not-construction"))
(DefineLink
   pln-rule-not-construction-name
   pln-rule-not-construction)
