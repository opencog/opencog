; P OR Q = Not(P) => Q

(define P (ConceptNode "P"))
(define Q (ConceptNode "Q"))

(OrLink P Q (stv 0.1 0.1))

(EvaluationLink (PredicateNode "query")
(ListLink
 (SubsetLink
    (NotLink P)
    Q
 )
)
)

(EvaluationLink (PredicateNode "rules") (ListLink
    (ConceptNode "BooleanTransformationRule<OrLink,SubsetLink>")
))

