; ----- For SuReal ----- ;
(nlp-parse "cats can read")
; (nlp-parse "A dog can eat")

; ----- PLN trail ----- ;
(define rb (ConceptNode "r2l-pln"))

(define inst1-rule
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

(define inst2-rule
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
