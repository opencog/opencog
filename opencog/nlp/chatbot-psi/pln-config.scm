(use-modules (opencog))
(use-modules (opencog atom-types))
(use-modules (opencog pln))
(use-modules (opencog rule-engine))

;-------------------------------------------------------------------------------
; Define a rule-base.
(define rb (ConceptNode "r2l-pln"))

(define sog-hack-decomposition-rule
  (Bind
    (VariableList
      (TypedVariable
        (Variable "$W")
        (Type "WordInstanceNode"))
      (TypedVariable
        (Variable "$P")
        (Type "PredicateNode"))
      (TypedVariable
        (Variable "$A")
        (Type "ConceptNode"))
      (TypedVariable
        (Variable "$B")
        (Type "ConceptNode"))
      (TypedVariable
        (Variable "$A-subset")
        (Type "ConceptNode")))
    (And
      ; The ReferenceLink and the InheritanceLink specify the set specified
      ; by the the predicate (Variable "$P").
      (Reference
        (Variable "$P")
        (Variable "$W"))
      (Inheritance
        (Variable "$A-subset")
        (Variable "$A"))
      (Evaluation
        (Variable "$P")
        (List
          (Variable "$A")
          (Variable "$B"))))
    (Evaluation (stv 1 1)
      (Variable "$P")
      (List
        (Variable "$A-subset")
        (Variable "$B")))))


(define (configure-pln)
    (pln-load-rules "deduction")
    (pln-load-rules "abduction")

    ; NOTE: The number has no relevance in r2l-mode
    (ure-define-rbs rb 0)
    ; Not sure why
    (ure-set-fuzzy-bool-parameter rb "URE:attention-allocation" 0)

    (ure-define-add-rule rb "abduction-inheritance-rule"
         abduction-inheritance-rule .8)
    (ure-define-add-rule rb "deduction-inheritance-rule"
        deduction-inheritance-rule .8)
    (ure-define-add-rule rb "sog-hack-decomposition-rule"
        sog-hack-decomposition-rule .8)
)

(define (infer-on-r2l r2l-outputs steps)
    (configure-pln)
    (let* ((inference-results (simple-forward-chain rb r2l-outputs steps))
          (clean-results (lset-difference equal? inference-results r2l-outputs))
          (results-for-sureal
              (cog-outgoing-set (filter-for-sureal clean-results)))
          )
        results-for-sureal
    )
)
