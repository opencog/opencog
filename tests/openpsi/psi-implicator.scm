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

(define (component-1) (psi-component "component-1"))

; FIXME: Using psi-goal results in the failure of OpenPsiRulesUTest
; and OpenPsiImplicatorUTest
;(define goal-1 (psi-goal "goal-1" 1))
(define goal-1 (Concept "goal-1"))

(define (rule-1) (psi-rule context-1 action-1 goal-1 (stv 1 1) (component-1)))

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
(define (rule-2) (psi-rule context-2 action-1 goal-1 (stv 1 1) (component-1)))

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

; Rule-3 is ungroundable
(define context-3
  (append context-1 (list
    (Link (Variable "a") (Variable "b")))))

(define (context-3-cpp) (List context-3))

(define test-pred-rtn #f)
(define-public (test-pred)
  (if test-pred-rtn (stv 1 1) (stv 0 1)))

(define context-4
  (list (Satisfaction
    (And (Evaluation (GroundedPredicate "scm: test-pred") (List))))))

(define (context-4-cpp) (List context-4))

(define action-2 (True))
