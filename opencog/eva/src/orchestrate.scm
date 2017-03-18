;
; orchestrate.scm
;
; Action orchestrator, mini version - expression multiplexer.
;
; This implements an extremely simple, no-frills action orchestrator
; for facial expressions and gestures.  It acts as a "multiplexer":
; allows different subsystems to ask for different expressions to be
; displayed, but, in the end, only allowing one or two of these to
; get through at any given time.
;
; The idea is that one subsystem may want to make the robot smile;
; another might want to make the robot frown; it cannot smile and
; frown at the same time, so only one expression should be displayed.
; The other expression can be either delayed, or completely ignored
; (rejected).  It is the code here, the "action orchestrator", that
; maintains this control and sequencing.
;
; An example: the language-processing code might get a command:
; "please smile for me", while the behavior tree might be issuing
; a frown expression. (Not clear which should have precedence).
;
; All requests need to include the name of the subsystem making the
; request. This name is currently ignored, but it is planned that
; it will be used to determine priority, and/or to block requests
; from certain sources.
;
; -------------------------------------------------------------


; -------------------------------------------------------------
; Request a display of a facial expression (smile, frown, etc.)
; The expression name should be one of the supported blender animations.
;
; Example usage:
;    (cog-evaluate! (Put (DefinedPredicate "Show facial expression")
;         (ListLink (Concept "happy") (Number 6) (Number 0.6))))
;
; As of right now, there is nothing to "orchestrate" here, since each
; animation fades out pretty quickly, and its very unlikely that we'll
; get conflicting expression directives at a rate of more than one
; every few seconds. So basically, we accept all directives, and show
; them immediately.
;
; If we wanted to rate-limit this, then make a copy of "change-template"
; and edit it to provide a minimum elapsed time predicate.
;
; XXX FIXME: this records the animation that was chosen, and a
; timestamp in some StateLinks. These need to be replaced by the
; TimeServer, instead.
;
(DefineLink
	(DefinedPredicate "Show facial expression")
	(LambdaLink
		(VariableList
			(Variable "$expr")
			(Variable "$duration")
			(Variable "$intensity"))
		(SequentialAndLink
			;; Record the time
			(TrueLink (DefinedSchema "set expression timestamp"))
			;; Record the expression itself
			(TrueLink (State face-expression-state (Variable "$expr")))
			;; Send it off to ROS to actually do it.
			(EvaluationLink (GroundedPredicate "py:do_face_expression")
				(ListLink
					(Variable "$expr")
					(Variable "$duration")
					(Variable "$intensity")))
		)))

; -------------------------------------------------------------
; Request a display of a facial gesture (blink, nod, etc.)
; The expression name should be one of the supported blender animations.
;
; Example usage:
;    (cog-evaluate! (Put (DefinedPredicate "Show gesture")
;         (ListLink (Concept "blink") (Number 0.8) (Number 3) (Number 1))))
;
(DefineLink
	(DefinedPredicate "Show gesture")
	(LambdaLink
		(VariableList
			(Variable "$gest")
			(Variable "$insensity")
			(Variable "$repeat")
			(Variable "$speed"))
		(SequentialAndLink
			;; Log the time.
			(True (DefinedSchema "set gesture timestamp"))
			;; Send it off to ROS to actually do it.
			(EvaluationLink (GroundedPredicate "py:do_gesture")
				(ListLink
					(Variable "$gest")
					(Variable "$insensity")
					(Variable "$repeat")
					(Variable "$speed")))
		)))

; -------------------------------------------------------------
; Request robot to look at a specific coordinate point.
; Currently, a very thin wrapper around py:look_at_point

(DefineLink
	(DefinedPredicate "Look at point")
	(LambdaLink
		(VariableList (Variable "$x") (Variable "$y") (Variable "$z"))
		(SequentialAndLink
			;; Log the time.
			; (True (DefinedSchema "set gesture timestamp"))
			;; Send it off to ROS to actually do it.
			(EvaluationLink (GroundedPredicate "py:look_at_point")
				(ListLink (Variable "$x") (Variable "$y") (Variable "$z")))
		)))

;---------------------------------------------------------------

; Request robot to turn eyes at a specific coordinate point.
; Currently, a very thin wrapper around py:gaze_at_point

(DefineLink
	(DefinedPredicate "Gaze at point")
	(LambdaLink
		(VariableList (Variable "$x") (Variable "$y") (Variable "$z"))
		(SequentialAndLink
			;; Log the time.
			; (True (DefinedSchema "set gesture timestamp"))
			;; Send it off to ROS to actually do it.
			(EvaluationLink (GroundedPredicate "py:gaze_at_point")
				(ListLink (Variable "$x") (Variable "$y") (Variable "$z")))
		)))

; -------------------------------------------------------------
; As above, but (momentarily) break eye contact, first.
; Otherwise, the behavior tree forces eye contact to be continually
; running, and the turn-look command is promptly over-ridden.
; XXX FIXME, this is still broken during search for attention.

(DefineLink
	(DefinedPredicate "Look command")
	(LambdaLink
		(VariableList (Variable "$x") (Variable "$y") (Variable "$z"))
		(SequentialAndLink
			(DefinedPredicate "break eye contact")
			(EvaluationLink (DefinedPredicate "Gaze at point")
				(ListLink (Variable "$x") (Variable "$y") (Variable "$z")))
			(EvaluationLink (DefinedPredicate "Look at point")
				(ListLink (Variable "$x") (Variable "$y") (Variable "$z")))
		)))

(DefineLink
	(DefinedPredicate "Gaze command")
	(LambdaLink
		(VariableList (Variable "$x") (Variable "$y") (Variable "$z"))
		(SequentialAndLink
			(DefinedPredicate "break eye contact")
			(EvaluationLink (DefinedPredicate "Gaze at point")
				(ListLink (Variable "$x") (Variable "$y") (Variable "$z")))
		)))

; The language-subsystem can understand commands such as "look at me"
; or, more generally, "look at this thing". At the moment, the only
; thing we can look at are faces, and the "salient point"
(DefineLink
	(DefinedPredicate "Look-at-thing cmd")
	(LambdaLink
		(Variable "$object-id")
		(SequentialOr
			(SequentialAnd
				(Equal (Variable "$object-id") (Concept "salient-point"))
				(DefinedPredicate "look at salient point"))
			(Evaluation
				(DefinedPredicate "Set interaction target")
				(ListLink (Variable "$object-id")))
		)))

; -------------------------------------------------------------
; Publish the current behavior.
; Cheap hack to allow external ROS nodes to know what we are doing.
; The string name of the node is sent directly as a ROS String message
; to the "robot_behavior" topic.
;
; Example usage:
;    (cog-evaluate! (Put (DefinedPredicate "Publish behavior")
;         (ListLink (Concept "foobar joke"))))
;
(DefineLink
	(DefinedPredicate "Publish behavior")
	(LambdaLink
		(VariableList (Variable "$bhv"))
		;; Send it off to ROS to actually do it.
		(EvaluationLink (GroundedPredicate "py:publish_behavior")
			(ListLink (Variable "$bhv")))
		))

; -------------------------------------------------------------
; Request to change the soma state.
; Takes two arguments: the requestor, and the proposed state.
;
; Currently, this always honors all requests.
; Currently, the requestor is ignored.
;
; Some future version may deny change requests, depending on the
; request source or on other factors.

(DefineLink
	(DefinedPredicate "Request Set Soma State")
	(LambdaLink
		(VariableList
			(Variable "$requestor")
			(Variable "$state"))
		(True (State soma-state (Variable "$state")))
	))

; -------------------------------------------------------------
; Request to change the facial expression state.
; Takes two arguments: the requestor, and the proposed state.
;
; Currently, this always honors all requests.
; Currently, the requestor is ignored.
;
; XXX Currently, this does nothing at all. Some future version may
; deny change requests, depending on the request source or on other
; factors.  XXX This is incompletely thought out and maybe should be
; removed.

(DefineLink
	(DefinedPredicate "Request Set Face Expression")
	(LambdaLink
		(VariableList
			(Variable "$requestor")
			(Variable "$state"))
		(True)
	))

; -------------------------------------------------------------
; Say "Hello recog-id"
; XXX FIXME this is totally wrong, this belongs in the chat interface,
; and not here!  That is, the proper greeting needs to be assembled
; elsewhere, and then use the standard chat interfaces!
(DefineLink
	(DefinedPredicate "Greet recognized person")
	(LambdaLink
		(VariableList (VariableNode "face-id") (VariableNode "recog-id"))
		(EvaluationLink
			(GroundedPredicate "py: greet_recognized_face")
			(ListLink
				(VariableNode "recog-id")))))


; -------------------------------------------------------------
; Say something. To test run,
; (cog-evaluate! (Put (DefinedPredicate "Say") (Node "this is a test"))))
(DefineLink
	(DefinedPredicate "Say")
	(LambdaLink (Variable "sentence")
		(Evaluation
			(GroundedPredicate "py: say_text")
			(List (Variable "sentence")))
	))

;Salient
(DefineLink
	(DefinedPredicate "look at salient point")
	(SequentialAnd
		(True (Put
			(Evaluation (DefinedPredicate "Look at point")
				(List (Variable "$x") (Variable "$y") (Variable "$z")))
			(Get (State salient-loc
				(List (Variable "$x") (Variable "$y") (Variable "$z"))))
		))
		(True (Put
			(Evaluation (DefinedPredicate "Gaze at point")
				(List (Variable "$x") (Variable "$y") (Variable "$z")))
			(Get (State salient-loc
				(List (Variable "$x") (Variable "$y") (Variable "$z"))))
		))
	))


; -------------------------------------------------------------
; For updating web-ui
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
*unspecified*  ; Make the load be silent
