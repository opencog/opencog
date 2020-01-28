(use-modules (opencog))
(use-modules (opencog nlp))
(use-modules (opencog pln))
(use-modules (opencog ure))

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
    (define rb (ConceptNode "r2l-pln-2"))

    ; TODO: use pln-load-rules when move to new PLN API, see
    ; https://github.com/opencog/pln/blob/master/opencog/pln/README.md
    (load-from-path (pln-rule-type->filename "term/deduction"))
    (load-from-path (pln-rule-type->filename "wip/abduction"))

    ; NOTE: The number has no relevance in r2l-mode
    (ure-define-rbs rb 0)
    ; Not sure why
    (ure-set-fuzzy-bool-parameter rb "URE:attention-allocation" 0)

    (ure-define-add-rule rb "abduction-inheritance-rule"
         abduction-inheritance-rule (stv 1 1))
    (ure-define-add-rule rb "deduction-inheritance-rule"
        deduction-inheritance-rule (stv 1 1))
    (ure-define-add-rule rb "sog-hack-decomposition-rule"
        sog-hack-decomposition-rule (stv 1 1))

    ; Return the rule-base
    rb
)

;; Define rulebases
(define rb-trail-2 (configure-pln-rbs-2))
