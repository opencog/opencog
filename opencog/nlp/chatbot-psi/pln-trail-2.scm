(use-modules (opencog))
(use-modules (opencog atom-types))
(use-modules (opencog pln))
(use-modules (opencog rule-engine))

;-------------------------------------------------------------------------------
; Define a rule-base for a particular infernece-trail.
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

; TODO: Record the acutal inference trails that have been learned and apply
; them instead of trying to randomly try different permutations.
(define (configure-pln-rbs-2)
    (define rb (ConceptNode "r2l-pln-1"))

    (pln-load-rules "term/deduction")
    (pln-load-rules "wip/abduction")

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

    ; Return the rule-base
    rb
)

;; Define rulebases
(define rb-trail-2 (configure-pln-rbs-2))
