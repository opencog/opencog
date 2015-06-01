(define true (stv 1.0 0.1))
(define false (stv 0.0 0.1))

(define A (PredicateNode "A"))
(define B (PredicateNode "B"))

(define one (ConceptNode "1"))
(define two (ConceptNode "2"))
(define three (ConceptNode "3"))
(define four (ConceptNode "4"))

(EvaluationLink A (ListLink one two) true)
(EvaluationLink A (ListLink one four) true)
(EvaluationLink A (ListLink three four) true)

(EvaluationLink B (ListLink one two) true)
(EvaluationLink B (ListLink one four) true)
(EvaluationLink B (ListLink three four) false)
(EvaluationLink B (ListLink four four) true)

(EvaluationLink (PredicateNode "query") (ListLink
    (AndLink A B)
))

(EvaluationLink (PredicateNode "rules") (ListLink
    (ConceptNode "AndBulkEvaluationRule<2>")
))

