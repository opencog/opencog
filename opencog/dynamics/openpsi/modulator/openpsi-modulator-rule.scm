
(define psi-modulator-updater-rule
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "Modulator")
                (TypeNode "ConceptNode")
            )
        )
        (AndLink
            (InheritanceLink
                (VariableNode "Modulator")
                (ConceptNode "OpenPsi: Modulator")
            )
            ; Why this link?-> To enable flexibility by allowing the definition
            ; of other rules.
            (EvaluationLink
                (PredicateNode "Psi: Stimulates")
                (ListLink
                    (GroundedSchemaNode "scm: psi-modulator-updater")
                    (VariableNode "Modulator")
                )
            )
        )
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: psi-modulator-updater") ; pre-condition
            (ListLink
                (VariableNode "Modulator")
            )
        )
    )
)

(define (psi-modulator-updater modulator)
    (let ((modulator-name (cog-name  modulator)))
         (cond
             ((equal? modulator-name "OpenPsi: Activation")
                (cog-set-tv! modulator (stv (ActivationModulatorUpdater) 1))
             )
         )
     )
     #t
)
