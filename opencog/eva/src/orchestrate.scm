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
;    (cog-evaluate! (Put (DefinedPredicate "Show expression")
;         (ListLink (Concept "happy") (Number 6) (Number 0.6))))
;
; As of right now, there is nothing to "orchestrate" here, since each
; animation fades out pretty quickly (the "duration" seems to be
; ignored!?) and its very unlikely that we'll get conflicting
; expression directives at a rate of more than one every few seconds.
; So basically, we accept all directives, and show them immedaitely.
;
; If we wanted to rate-limit this, then make a copy of "change-template"
; and edit it to provide a minimum elapsed time predicate.
;
(DefineLink
	(DefinedPredicate "Show expression")
	(LambdaLink
		(VariableList
			(Variable "$expr")
			(Variable "$duration")
			(Variable "$intensity"))
		(SequentialAndLink
			;; Record the time
			(TrueLink (DefinedSchema "set expression timestamp"))
			;; Send it off to ROS to actually do it.
			(EvaluationLink (GroundedPredicate "py:do_emotion")
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

; -------------------------------------------------------------
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
; Otherwise, the behavior tree forces eye contact to be continueally
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
; Request to change the emotion state.
; Takes two arguments: the requestor, and the proposed state.
;
; Currently, this always honors all requests.
; Currently, the requestor is ignored.
;
; Some future version may deny change requests, depending on the
; request source or on other factors.

(DefineLink
	(DefinedPredicate "Request Set Emotion State")
	(LambdaLink
		(VariableList
			(Variable "$requestor")
			(Variable "$state"))
		(True (State emotion-state (Variable "$state")))
	))

; -------------------------------------------------------------
; Say "Hello recog-id"
; TODO: Add a request to chat psi-rules. That way actions of psi-rules will
; be composable by the planner/action-orchestrator.
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
;show happy emotion	
(DefineLink
    (DefinedPredicate "Quiet")
    (LambdaLink (Concept "happy")
         (Evaluation
            (GroundedPredicate "py: do_emotion")
           (List (Concept "happy")(NumberNode 3) (NumberNode 0.5)))
    )
; -------------------------------------------------------------
*unspecified*  ; Make the load be silent
