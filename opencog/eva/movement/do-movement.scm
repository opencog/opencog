;
; do-movement.scm
;
; Implement the movement API for ROS/blender animations.
;

; Delete the current definition, if any.
(define (delete-definition STR)
	(define dfn
		(cog-get-link 'DefineLink 'DefinedPredicateNode
			(DefinedPredicate STR)))

	(if (not (null? dfn)) (cog-delete (car dfn)) #f))

; -------------------------------------------------------------
; Request a display of a facial expression (smile, frown, etc.)
; The expression name should be one of the supported blender animations.
;
; Example usage:
;    (cog-evaluate! (Put (DefinedPredicate "Show facial expression")
;         (ListLink (Concept "happy") (Number 6) (Number 0.6))))
;
(delete-definition "Do show facial expression")
(DefineLink
	(DefinedPredicate "Do show facial expression")
	(LambdaLink
		(VariableList
			(Variable "$expr")
			(Variable "$duration")
			(Variable "$intensity"))
		(SequentialAndLink
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
(delete-definition "Do show gesture")
(DefineLink
	(DefinedPredicate "Do show gesture")
	(LambdaLink
		(VariableList
			(Variable "$gest")
			(Variable "$insensity")
			(Variable "$repeat")
			(Variable "$speed"))
		(SequentialAndLink
			;; Send it off to ROS to actually do it.
			(EvaluationLink (GroundedPredicate "py:do_gesture")
				(ListLink
					(Variable "$gest")
					(Variable "$insensity")
					(Variable "$repeat")
					(Variable "$speed")))
		)))

; -------------------------------------------------------------
; Eye-saccade control.
; (cog-evaluate! (Put (DefinedPredicate "Say") (Node "this is a test"))))

(delete-definition "Conversational Saccade")
(delete-definition "Listening Saccade")
(delete-definition "Explore Saccade")

(DefineLink
	(DefinedPredicate "Conversational Saccade")
	(LambdaLink
		(Evaluation
			(GroundedPredicate "py: conversational_saccade")
			(List))
	))

(DefineLink
	(DefinedPredicate "Listening Saccade")
	(LambdaLink
		(Evaluation
			(GroundedPredicate "py: listening_saccade")
			(List))
	))

(DefineLink
	(DefinedPredicate "Explore Saccade")
	(LambdaLink
		(Evaluation
			(GroundedPredicate "py: explore_saccade")
			(List))
	))

; -------------------------------------------------------------
; Control the blink rate of the robot.

(delete-definition "Blink rate")
(DefineLink
	(DefinedPredicate "Blink rate")
	(LambdaLink
		(VariableList (Variable "$mean") (Variable "$var"))
		(SequentialAndLink
			;; Send it off to ROS to actually do it.
			(EvaluationLink (GroundedPredicate "py: blink_rate")
				(ListLink (Variable "$mean") (Variable "$var")))
		)))

; -------------------------------------------------------------
; Request robot to look at a specific coordinate point.
; Currently, a very thin wrapper around py:look_at_point

(delete-definition "Do look at point")
(DefineLink
	(DefinedPredicate "Do look at point")
	(LambdaLink
		(VariableList (Variable "$x") (Variable "$y") (Variable "$z"))
		(SequentialAndLink
			;; Send it off to ROS to actually do it.
			(EvaluationLink (GroundedPredicate "py:look_at_point")
				(ListLink (Variable "$x") (Variable "$y") (Variable "$z")))
		)))

;---------------------------------------------------------------

; Request robot to turn eyes at a specific coordinate point.
; Currently, a very thin wrapper around py:gaze_at_point

(delete-definition "Do gaze at point")
(DefineLink
	(DefinedPredicate "Do gaze at point")
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
; Publish the current behavior.
; Cheap hack to allow external ROS nodes to know what we are doing.
; The string name of the node is sent directly as a ROS String message
; to the "robot_behavior" topic.
;
; Example usage:
;    (cog-evaluate! (Put (DefinedPredicate "Publish behavior")
;         (ListLink (Concept "foobar joke"))))
;
(delete-definition "Publish behavior")
(DefineLink
	(DefinedPredicate "Publish behavior")
	(LambdaLink
		(VariableList (Variable "$bhv"))
		;; Send it off to ROS to actually do it.
		(EvaluationLink (GroundedPredicate "py: publish_behavior")
			(ListLink (Variable "$bhv")))
		))

; -------------------------------------------------------------

; Call once, to fall asleep.
(delete-definition "Do go to sleep")
(DefineLink
	(DefinedPredicate "Do go to sleep")
	; Play the go-to-sleep animation.
	(Evaluation (GroundedPredicate "py:do_go_sleep") (ListLink))
)

; Wake-up sequence
(delete-definition "Do wake up")
(DefineLink
	(DefinedPredicate "Do wake up")

	; Run the wake animation.
	(Evaluation (GroundedPredicate "py:do_wake_up") (ListLink))
)

; -------------------------------------------------------------
; Say something. To test run,
; (cog-evaluate! (Put (DefinedPredicate "Say") (Node "this is a test"))))
(delete-definition "Say")
(DefineLink
	(DefinedPredicate "Say")
	(LambdaLink (Variable "sentence")
		(Evaluation
			(GroundedPredicate "py: say_text")
			(List (Variable "sentence")))
	))

; -------------------------------------------------------------
; Return true if ROS is still running.
(delete-definition "ROS is running?")
(DefineLink
	(DefinedPredicate "ROS is running?")
	(Evaluation
		(GroundedPredicate "py:ros_is_running") (ListLink)))

; -------------------------------------------------------------
*unspecified*  ; Make the load be silent
