; Basic ForAll demo using Implication. It has to be with InheritanceRule to prevent ModusPonensRule from triggering.

(define tv (stv 1 0.999))

(define A (ConceptNode "A"))
(define B (ConceptNode "B"))
(define C (ConceptNode "C"))
(define x001 (VariableNode "x001"))

(ForAllLink tv (ListLink x001)
    (InheritanceLink
        x001
        B
    )
)

(define target
    (InheritanceLink
        B
        C
    )
)

