; Energy psi-demand definition
(InheritanceLink
    ; The strength of the stv is the demand-value, & confidence is always 1.
    (ConceptNode "OpenPsi: Energy" (stv .71 1))
    (ConceptNode "OpenPsi: Demand")
)


(EvaluationLink
    (PredicateNode "must_have_value_within")
    (ListLink
        (ConceptNode "OpenPsi: Energy")
        (NumberNode .5)
        (NumberNode .7)
    )
)

; Actions that can affect the psi-demand Energy-
; There could be multiple actions, which one is choosen to be run is determined
; by the action selection agent, with the exception of 'psi-demand-updater',
; that is always running.
; For an action to be included it should be able to manipulate the demand-value
(EvaluationLink
    (PredicateNode "Psi: acts-on")
    (ListLink
        ; This is the default function that each psi-demand must have.
        (GroundedSchemaNode "scm: psi-demand-updater")
        (ConceptNode "OpenPsi: Energy")
    )
)
