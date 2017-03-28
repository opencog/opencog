;; To be replaced by negation-introduction-rule 

; =====================================================================
; NotConstructionRule
; (http://wiki.opencog.org/w/NotConstructionRule TODO)
;
; A<TV1>
; |-
; NotLink <TV>
;    A
;----------------------------------------------------------------------

(define not-construction-rule
  (BindLink
     (VariableList
        (TypedVariableLink
           (VariableNode "$A")
           (TypeChoice
              (TypeNode "PredicateNode")
              (TypeNode "ConceptNode"))))
     (VariableNode "$A")
     (ExecutionOutputLink
        (GroundedSchemaNode "scm: not-construction-formula")
        (ListLink
           (VariableNode "$A")))))

(define (not-construction-formula A)
  (cog-set-tv!
   (NotLink A)
   (not-construction-side-effect-free-formula A))
)

(define (negate x)
  (- 1 x))

(define (not-construction-side-effect-free-formula A)
  (let ((sA (cog-stv-strength A))
        (cA (cog-stv-confidence A)))
    (stv (negate sA) cA)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Some test data (to be removed afterwards) ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Name the rule
(define not-construction-rule-name
  (DefinedSchemaNode "not-construction-rule"))
(DefineLink
   not-construction-rule-name
   not-construction-rule)
