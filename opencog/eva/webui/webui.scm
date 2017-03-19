;
; webui.scm
;
; User Interface for OpenPsi control

(define-module (opencog webui))

(use-modules (opencog) (opencog atom-types) (opencog openpsi))

;
; NOTE: updating of parameters is divided into steps of upating the parameter
; cache and then pushing the update, so as to simply syncing the values.
; If one pushes a partial updated cache results in the publishing of the change
; to /opencog_control/parameter_updates topic thus resulting in an undesirable
; state in the atomspace.

; Update dynamic parameter cache
(DefineLink
    (DefinedPredicate "update-opencog-control-parameter")
    (LambdaLink
        (VariableList
            (TypedVariableLink
                (VariableNode "psi-rule-alias")
                (TypeNode "ConceptNode"))
            (TypedVariableLink
                (VariableNode "psi-rule-weight")
                (TypeNode "NumberNode")))
        (Evaluation
            (GroundedPredicate "py: update_opencog_control_parameter")
            (List
                (VariableNode "psi-rule-alias")
                (VariableNode "psi-rule-weight")))
    ))

; Push dynamic parameter cache values
(Define
	(DefinedPredicate "push-parameter-update")
	(Evaluation
		(GroundedPredicate "py: push_parameter_update")
		(List))
	)

; This is needed as the parameters (stored in ros_commo/param_dict)
; may be updated by a separate thread, so doing this is to make
; sure that the full set of parameters will only be pushed when the
; thread has finished the update, so as to avoid having half-updated
; set of parameters pushed (and as a result re-applied to opencog via
; the msg being published on /opencog_control/parameter_updates)
(Define
	(DefinedPredicate "parameter-update-is-done")
	(Equal
		(Set psi-controller-idle)
		(Get (State psi-controller (Variable "$x"))))
)

(Define
	(DefinedPredicate "update-web-ui")
	(SequentialAnd
		(True (PutLink
			(DefinedPredicate "update-opencog-control-parameter")
			(DefinedSchema "psi-controlled-rule-state")))
		(DefinedPredicate "parameter-update-is-done")
		(DefinedPredicate "push-parameter-update")
	))

; -------------------------------------------------------------
; Now hook it up.

; Any changes to the weight for controlled-psi-rules are pushed to
; ros dynamic-parameters. Thus the web-ui mirrors the opencog
; wholeshow state.
(psi-rule (list (DefinedPredicate "ROS is running?"))
	(DefinedPredicate "update-web-ui")
		update-demand-satisfied (stv 1 1) update-demand)

; -------------------------------------------------------------
*unspecified*  ; Make the load be silent
