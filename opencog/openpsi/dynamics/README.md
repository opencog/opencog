## Modulators and Internal Dynamics

OpenPsi includes a dynamic system of interacting modulator variables.
The model contains varying parameters that regulate processes such as
stimulus evaluation, action selection, and emotional expression.
Modulators are based on MicroPsi, and SECs are based on Component
Process Model theory (see [references](#References) for more info). These and other
OpenPsi-related entities interact with each other according
to rules that specify how a change in a "trigger" entity cause a change
in a "target" entity.

The interaction rules have the logical form of:

    change in trigger entity --> change in target entity

The change in the target is a function of the magnitude of change in the
trigger, the strength of the interaction rule, and the current value of
the target.

The rules in Atomese have the form:

    (PredictiveImplication
        (TimeNode 1)
            (Evaluation
                  (Predicate "psi-changed")
                  (List
                      trigger))
            (ExecutionOutputLink
                (GroundedSchema "scm: adjust-psi-var-level")
                (List
                    target
                    (NumberNode strength)
                    trigger))))

The antecedent predicate can be one of "psi-changed", "psi-increased"
or "psi-decreased".

Files:

* updater.scm - the main control file that handles the dynamic updating
  of entity values based on the interaction rules.
* interaction-rule.scm - code for creating the rules specifying
  interaction dynamics between entities.
* modulator.scm - internal openpsi modulator variables
* sec.scm - internal openpsi Stimulus Evaluation Check variables
* events.scm - defines perceived events that are monitored and that
  trigger changes in openpsi variables.
* emotion.scm - code for creating emotions

Instructions:

* Create interaction rules with the psi-create-interaction-rule function in
  interaction-rule.scm
* Create monitored events with psi-create-monitored-event function in event.scm.
* Create event detection callback(s) with (psi-set-event-callback! callback) in
  updater.scm. The callback function should indicate that a particular event has
  occurred by calling (psi-set-event-occurrence! event), which uses a StateLink
  to represent the most recent occurrence of the event.
* Create "expression" callback function with (psi-set-expression-callback!
  callback) in updater.scm. This function should handle implementation-specific
  expression of emotion and/or physiology based on the OpenPsi dynamic variables.

See [this](../../eva/src/psi-dynamics.scm) for an example of using this model,

TODO: Create general callback that is called at each loop step.

