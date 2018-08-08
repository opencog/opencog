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
			(EvaluationLink (DefinedPredicate "Do show facial expression")
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
			(EvaluationLink (DefinedPredicate "Do show gesture")
				(ListLink
					(Variable "$gest")
					(Variable "$insensity")
					(Variable "$repeat")
					(Variable "$speed")))
		)))

; -------------------------------------------------------------
; Request robot to look at a specific coordinate point.
; Currently, a very thin wrapper around "Do look at point"

(DefineLink
	(DefinedPredicate "Look at point")
	(LambdaLink
		(VariableList (Variable "$x") (Variable "$y") (Variable "$z"))
		(SequentialAndLink
			;; Log the time.
			; (True (DefinedSchema "set gesture timestamp"))
			;; Send it off to ROS to actually do it.
			(EvaluationLink (DefinedPredicate "Do look at point")
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
			(EvaluationLink (DefinedPredicate "Do gaze at point")
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

; Look at the Salient point
(define salient-loc  (AnchorNode "Salient location"))
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

; Call once, to fall asleep.
(DefineLink
	(DefinedPredicate "Go to sleep")
	(SequentialAnd
		; Proceed only if we are allowed to.
		(Put (DefinedPredicate "Request Set Face Expression")
			(ListLink bhv-source (ConceptNode "sleepy")))

		; Proceed with the sleep animation only if the state
		; change was approved.
		(Evaluation (DefinedPredicate "Request Set Soma State")
			(ListLink bhv-source soma-sleeping))

		(Evaluation (GroundedPredicate "scm: print-msg-time")
			(ListLink (Node "--- Go to sleep.")
				(Minus (TimeLink) (DefinedSchema "get bored timestamp"))))
		(True (DefinedSchema "set sleep timestamp"))

		(Put (DefinedPredicate "Publish behavior")
			(Concept "Falling asleep"))

		; First, show some yawns ...
		(Put (DefinedPredicate "Show random gesture")
			(Concept "sleepy"))

		; Finally, play the go-to-sleep animation.
		(Evaluation (DefinedPredicate "Do go to sleep") (ListLink))
	))

; Wake-up sequence
(DefineLink
	(DefinedPredicate "Wake up")
	(SequentialAnd
		; Request change soma state to being awake. Proceed only if
		; the request is accepted.
		(Evaluation (DefinedPredicate "Request Set Soma State")
			(ListLink bhv-source soma-awake))

		; Proceed only if we are allowed to.
		(Put (DefinedPredicate "Request Set Face Expression")
			(ListLink bhv-source (ConceptNode "wake-up")))

		(Evaluation (GroundedPredicate "scm: print-msg-time")
			(ListLink (Node "--- Wake up!")
				(Minus (TimeLink) (DefinedSchema "get sleep timestamp"))))

		(Put (DefinedPredicate "Publish behavior")
			(Concept "Waking up"))

		; Reset the bored timestamp, as otherwise we'll fall asleep
		; immediately (cause we're bored).
		(True (DefinedSchema "set bored timestamp"))

		; Reset the "heard something" state and timestamp.
		(True (DefinedPredicate "Heard Something?"))
		(True (DefinedSchema "set heard-something timestamp"))

		; Run the wake animation.
		(Evaluation (DefinedPredicate "Do wake up") (ListLink))

		; Also show the wake-up expression (head shake, etc.)
		(Put (DefinedPredicate "Show random expression")
			(Concept "wake-up"))
		(Put (DefinedPredicate "Show random gesture")
			(Concept "wake-up"))
	))

; -------------------------------------------------------------
*unspecified*  ; Make the load be silent
