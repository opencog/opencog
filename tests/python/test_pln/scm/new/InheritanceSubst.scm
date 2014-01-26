; simple test of InheritanceSubstRule
(define A (ConceptNode "A"))
(define B (ConceptNode "B"))
(define R (InheritanceLink (stv 0.9 0.9) A B))
(define P (PredicateNode "P"))
(define C (EvaluationLink (stv 0.6 0.4) P (ListLink B)))

; inference
(define target (InheritanceSubstRule R C))

; to be testable by PLNSchemeWrapperUTest
target
