(define true (stv 1.0 0.1))
(define false (stv 0.0 0.1))

(define A (ConceptNode "A"))
(define B (ConceptNode "B"))

(MemberLink (ConceptNode "1") A true)
(MemberLink (ConceptNode "2") A true)
(MemberLink (ConceptNode "3") A true)
(MemberLink (ConceptNode "4") A true)

(MemberLink (ConceptNode "1") B true)
(MemberLink (ConceptNode "2") B false)
(MemberLink (ConceptNode "3") B true)
(MemberLink (ConceptNode "4") B false)
(MemberLink (ConceptNode "42") B true)

(EvaluationLink (PredicateNode "query") (ListLink
    (AndLink A B)
))

(EvaluationLink (PredicateNode "rules") (ListLink
    (ConceptNode "AndBulkEvaluationRule")
))

