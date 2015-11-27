
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
                (PredicateNode "Psi: Stimulus")
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

; Rule Name
(DefineLink (Node "psi-modulator-updater-rule") psi-modulator-updater-rule)

(define (psi-modulator-updater modulator)
    ; The strength of the stv is the stimulus level, & confidence is always 1.
    (let ((modulator-name (cog-name  modulator)))
         (cond
             ((equal? modulator-name "OpenPsi: Activation")
                (cog-set-tv! modulator (stv (ActivationModulatorUpdater) 1))
             )
             ((equal? modulator-name "OpenPsi: Resolution")
                (cog-set-tv! modulator (stv (ResolutionModulatorUpdater) 1))
             )
             ((equal? modulator-name "OpenPsi: SelectionThreshold")
                (cog-set-tv! modulator (stv
                    (SelectionThresholdModulatorUpdater) 1))
             )
             ((equal? modulator-name "OpenPsi: SecuringThreshold")
                (cog-set-tv! modulator (stv
                    (SecuringThresholdModulatorUpdater) 1))
             )
         )
     )
     #t
)
