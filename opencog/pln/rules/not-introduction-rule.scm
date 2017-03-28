;; To be replaced by negation-introduction-rule 

; =====================================================================
; Not introduction rule
;
; A<TV1>
; |-
; NotLink <TV>
;    A
;----------------------------------------------------------------------

(define not-introduction-rule
  (BindLink
     (VariableList
        (TypedVariableLink
           (VariableNode "$A")
           (TypeChoice
              (TypeNode "PredicateNode")
              (TypeNode "ConceptNode"))))
     (VariableNode "$A")
     (ExecutionOutputLink
        (GroundedSchemaNode "scm: not-introduction-formula")
        (ListLink
           (VariableNode "$A")))))

(define (not-introduction-formula A)
  (cog-set-tv!
   (NotLink A)
   (not-introduction-side-effect-free-formula A))
)

(define (negate x)
  (- 1 x))

(define (not-introduction-side-effect-free-formula A)
  (let ((sA (cog-stv-strength A))
        (cA (cog-stv-confidence A)))
    (stv (negate sA) cA)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Some test data (to be removed afterwards) ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Name the rule
(define not-introduction-rule-name
  (DefinedSchemaNode "not-introduction-rule"))
(DefineLink
   not-introduction-rule-name
   not-introduction-rule)
