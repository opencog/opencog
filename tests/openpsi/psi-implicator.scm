(use-modules (opencog) (opencog openpsi))

(define context-1
  (list
    (InheritanceLink
      (VariableNode "$H")
      (ConceptNode "human"))
    (EvaluationLink
      (Predicate "eat")
      (List
        (VariableNode "$H")
        (ConceptNode "Beso"))))
)

(define (context-1-cpp) (List context-1))

; Used for testing the case when the context is grounded by
; its alpha-equivalent pattern.
(define (context-1-alpha-equivalent)
  (list
    (InheritanceLink
      (VariableNode "$H2")
      (ConceptNode "human"))
    (EvaluationLink
      (Predicate "eat")
      (List
        (VariableNode "$H2")
        (ConceptNode "Beso"))))
)

(define action-1
  (InheritanceLink
    (VariableNode "$H")
    (ConceptNode "animal"))
)

(define (demand-1) (psi-demand  "demand-1"))

; TODO Replace with psi-goal.
(define goal-1 (Concept "goal-1"))

(define (rule-1) (psi-rule context-1 action-1 goal-1 (stv 1 1) (demand-1)))

(define (rule-1-cpp)
; Rule added to the atomspace not index.
  (ImplicationLink (stv 1 1)
    (AndLink
      (InheritanceLink
         (VariableNode "$H")
         (ConceptNode "human")
      )
      (EvaluationLink
         (PredicateNode "eat")
         (ListLink
            (VariableNode "$H")
            (ConceptNode "Beso")
         )
      )
      (InheritanceLink
         (VariableNode "$H")
         (ConceptNode "animal")
      )
    )
    (ConceptNode "goal-1")
  )
)

(define (pattern-link-1)
  ; Structure of the PatternLink created for checking satisfiablity.
  (PatternLink (And context-1))
)

(define context-2 (list (Satisfaction (And context-1 (True)))))

; A Ghost rule
(define (rule-2) (psi-rule context-2 action-1 goal-1 (stv 1 1) (demand-1)))

(define (context-2-cpp) (List context-2))

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
