
(define psi-demand-updater-rule
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "Demand")
                (TypeNode "ConceptNode")
            )
            (TypedVariableLink
                (VariableNode "min_acceptable_value")
                (TypeNode "NumberNode")
            )
            (TypedVariableLink
                (VariableNode "max_acceptable_value")
                (TypeNode "NumberNode")
            )
        )
        (AndLink
            (InheritanceLink
                ; the strength of the stv is the demand value
                (VariableNode "Demand")
                (ConceptNode "OpenPsi: Demand")
            )
            ; This is equivalent to an in-born drive/goal. When
            ; the value goes out of range then urge occurs
            (EvaluationLink
                (PredicateNode "must_have_value_within")
                (ListLink
                    (VariableNode "Demand")
                    (VariableNode "min_acceptable_value")
                    (VariableNode "max_acceptable_value")
                )
            )
            (EvaluationLink
                (PredicateNode "Psi: acts-on")
                (ListLink
                    (GroundedSchemaNode "scm: psi-demand-updater")
                    (VariableNode "Demand")
                )
            )
        )
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: psi-demand-updater") ; pre-condition
            (ListLink
                (VariableNode "Demand")
                (VariableNode "min_acceptable_value")
                (VariableNode "max_acceptable_value")
            )
        )
    )
)

; Rule Name
(DefineLink (Node "psi-demand-updater-rule") psi-demand-updater-rule)

(define (psi-demand-updater demand min-acceptable-value max-acceptable-value)
    (let ((current-value (tv-mean (cog-tv  demand)))
          (min-value (string->number (cog-name min-acceptable-value)))
          (max-value (string->number (cog-name max-acceptable-value)))
         )
         ; TODO:
         ;1. check the 'fuzzy_within' equations, it seems too have small effect.
         ;2. make it easier for different demands have differing updaters, as
         ;   different characters controled are likely have different
         ;   personality.
         (cog-set-tv! demand
             (stv (fuzzy_within current-value min-value max-value 100) 1)
         )
     )
     #t
)


; Not sure time-stamping if the current-value is the stv of the demand.
#!
(define (pre-demand-updater  DemandGoal min_acceptable_value
                             max_acceptable_value XxxDemandUpdater)
    (let ((current-demand-value ((cog-name XxxDemandUpdater))) ; runs updater
          (min-value (cog-name min_acceptable_value))
          (max-value (cog-name max_acceptable_value))
         )

         ; replaces set_modulator_or_demand_value function
         (cog-execute!
             (AssignLink
                 (TypeNode "AtTimeLink")
                 (ConceptNode "change to Timedomain node")
                 (TimeNode (current-time))
                 (ExecutionLink
                     XxxDemandUpdater
                     (ListLink) ; becuase the
                     (NumberNode current-demand-value)
                 )
             )
         )
         (fuzzy_within current-demand-value min-value max-value 100)
    )
    #t
)
!#
