; ----- For SuReal ----- ;
(nlp-parse "cats can read")

; NOTE: get-abstract-version could be used for this example but this workflow
; is closer to the generic approach in the long term.
(define r2l-post-processing-1-rule
  (Bind
    (VariableList
      (TypedVariable
        (Variable "$pred")
        (Type "PredicateNode"))
      (TypedVariable
        (Variable "$concept")
        (Type "ConceptNode"))
      (TypedVariable
        (Variable "$pred-inst")
        (Type "PredicateNode"))
      (TypedVariable
        (Variable "$concept-inst")
        (Type "ConceptNode")))
    (And
      (Evaluation
        (Variable "$pred-inst")
        (List (Variable "$concept-inst")))
      (Implication
        (Variable "$pred-inst")
        (Variable "$pred"))
      (Inheritance
        (Variable "$concept-inst")
        (Variable "$concept")))
    (Evaluation
      (Variable "$pred")
      (List (Variable "$concept"))))
)

(define r2l-post-processing-2-rule
  (Bind
    (VariableList
      (TypedVariable
        (Variable "$pred")
        (Type "PredicateNode"))
      (TypedVariable
        (Variable "$concept-1")
        (Type "ConceptNode"))
      (TypedVariable
        (Variable "$concept-2")
        (Type "ConceptNode"))
      (TypedVariable
        (Variable "$pred-inst")
        (Type "PredicateNode"))
      (TypedVariable
        (Variable "$concept-inst-1")
        (Type "ConceptNode"))
      (TypedVariable
        (Variable "$concept-inst-2")
        (Type "ConceptNode")))
    (And
      (Evaluation
        (Variable "$pred-inst")
        (List
          (Variable "$concept-inst-1")
          (Variable "$concept-inst-2")))
      (Implication
        (Variable "$pred-inst")
        (Variable "$pred"))
      (Inheritance
        (Variable "$concept-inst-1")
        (Variable "$concept-1"))
      (Inheritance
        (Variable "$concept-inst-2")
        (Variable "$concept-2")))
    (Evaluation
      (Variable "$pred")
      (List
        (Variable "$concept-1")
        (Variable "$concept-2"))))
)

(define r2l-post-processing-3-rule
  (Bind
    (VariableList
      (TypedVariable
        (Variable "$concept-1")
        (Type "ConceptNode"))
      (TypedVariable
        (Variable "$concept-2")
        (Type "ConceptNode")))
    (Evaluation
      (Predicate "are")
      (List
        (Variable "$concept-1")
        (Variable "$concept-2")))
    (Inheritance
      (Variable "$concept-1")
      (Variable "$concept-2")))
)

(define (configure-pln-rbs-3)
    (define rb (ConceptNode "r2l-pln-3"))

    ; NOTE: The number has no relevance in r2l-mode
    (ure-define-rbs rb 0)
    ; Not sure why
    (ure-set-fuzzy-bool-parameter rb "URE:attention-allocation" 0)

    (ure-define-add-rule rb "r2l-post-processing-1-rule"
         r2l-post-processing-1-rule (stv 1 1))
    (ure-define-add-rule rb "r2l-post-processing-2-rule"
         r2l-post-processing-2-rule (stv 1 1))
    (ure-define-add-rule rb "r2l-post-processing-3-rule"
         r2l-post-processing-3-rule (stv 1 1))

    ; Return the rule-base
    rb
)

;; Define rulebases
(define rb-trail-3 (configure-pln-rbs-3))
