; Energy psi-demand definition
(InheritanceLink
    ; The strength of the stv is the demand-value, & confidence is always 1.
    (ConceptNode "OpenPsi: Energy" (stv .71 1))
    (ConceptNode "OpenPsi: Demand")
)

; The
(EvaluationLink
    (PredicateNode "must_have_value_within")
    (ListLink
        (ConceptNode "OpenPsi: Energy")
        (NumberNode .5)
        (NumberNode .7)
    )
)

; actions that can act on the psi-demand Energy-
(EvaluationLink
    (PredicateNode "Psi: acts-on")
    (ListLink
        ; This is the default function that each psi-demand must have.
        (GroundedSchemaNode "scm: psi-demand-updater")
        (VariableNode "Demand")
    )
)
