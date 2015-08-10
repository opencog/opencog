; SecuringThreshold psi-modulator definition
(InheritanceLink
    ; The strength of the stv is the stimulus level, & confidence is always 1.
    (ConceptNode "OpenPsi: SecuringThreshold" (stv 0.5 1))
    (ConceptNode "OpenPsi: Modulator")
)

; Stimuli that can act on the psi-modulator SecuringThreshold
(EvaluationLink
    (PredicateNode "Psi: Stimulates")
    (ListLink
        ; This is the default function that each psi-modulator must have.
        (GroundedSchemaNode "scm: psi-modulator-updater")
        (ConceptNode "OpenPsi: SecuringThreshold")
    )
)
