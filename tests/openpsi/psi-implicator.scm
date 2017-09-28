(define (rule-1)
  (Implication
      (And
        (InheritanceLink
          (VariableNode "$H")
          (ConceptNode "human"))
        (EvaluationLink
          (Predicate "eat")
          (List
            (VariableNode "$H")
            (ConceptNode "Beso"))))
     (InheritanceLink
        (VariableNode "$H")
        (ConceptNode "animal"))))

(define (groundable-content-1)
; Some data to populate the atomspace for grounding (rule-1)
  (InheritanceLink
    (ConceptNode "Abeba")
    (ConceptNode "human"))
  (EvaluationLink
    (Predicate "eat")
    (List
      (ConceptNode "Abeba")
      (ConceptNode "Beso")))
  ;(InheritanceLink
  ;  (ConceptNode "Chala")
  ;  (ConceptNode "human"))
  ;(EvaluationLink
  ;  (Predicate "eat")
  ;  (List
  ;    (ConceptNode "Chala")
  ;    (ConceptNode "Beso")))
)

(define (test_imply_solution)
  (InheritanceLink
    (ConceptNode "Abeba")
    (ConceptNode "animal"))
)
