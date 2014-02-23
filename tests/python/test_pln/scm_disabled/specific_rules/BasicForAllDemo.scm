; Basic ForAll demo using Implication. It has to be with InheritanceRule to prevent ModusPonensRule from triggering.

(define tv (stv 1 0.999))

(define A (ConceptNode "A" tv))
(define B (ConceptNode "B" tv))
(define C (ConceptNode "C" tv))
(define x001 (VariableNode "x001" tv))

(ForAllLink tv (ListLink x001)
    (InheritanceLink
        x001
        B
    )
)

(InheritanceLink tv
    B
    C
)

(define target
    (InheritanceLink
        A
        C
    )
)

(EvaluationLink (PredicateNode "query") (ListLink target))
(EvaluationLink (PredicateNode "rules") (ListLink (ConceptNode "DeductionRule"))) ; also needs some quantifier rules that aren't done yet

