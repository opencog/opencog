;
; movement-api.scm
;
; Definitions providing the movement API.
;
; The definitions here give the API only, but not an implememtation.
; The only current implementation is in the module `(opencog movement)`
; which connects these to ROS blender API robot animations.

; Delete the current definition, if any.
(define (delete-definition STR)
	(define dfn
		(cog-get-link 'DefineLink 'DefinedPredicateNode
			(DefinedPredicate STR)))

	(if (not (null? dfn)) (cog-delete (car dfn)) #f))

; Printer stub
(define-public (prt-pred-defn PRED)
   (format #t "Called (DefinedPredicate ~a)\n" (cog-name PRED))
   (stv 1 1))

; Create a definition that is just a stub.
(define (dfn-pred PRED)
	(DefineLink
		PRED
		(EvaluationLink
			(GroundedPredicate "scm: prt-pred-defn")
			(ListLink PRED))))

;
; XXX FIXME: these record the animation that was chosen, and a
; timestamp in some StateLinks. These need to be replaced by the
; TimeServer, instead.
;
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
			(TrueLink)
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
			(True)
		)))

; -------------------------------------------------------------
; Eye-saccade control.
; Saying things should alter the saccade mode
;
; (cog-evaluate! (Put (DefinedPredicate "Say") (Node "this is a test"))))

(delete-definition "Conversational Saccade")
(delete-definition "Listening Saccade")
(delete-definition "Explore Saccade")

(dfn-pred (DefinedPredicate "Conversational Saccade"))
(dfn-pred (DefinedPredicate "Listening Saccade"))
(dfn-pred (DefinedPredicate "Explore Saccade"))

; -------------------------------------------------------------
; Control the blink rate of the robot.

(delete-definition "Blink rate")
(DefineLink
	(DefinedPredicate "Blink rate")
	(LambdaLink
		(VariableList (Variable "$mean") (Variable "$var"))
		(True)))

; -------------------------------------------------------------
; Request robot to look at a specific coordinate point.

(delete-definition "Do look at point")
(DefineLink
	(DefinedPredicate "Do look at point")
	(LambdaLink
		(VariableList (Variable "$x") (Variable "$y") (Variable "$z"))
		(SequentialAndLink
			(True)
		)))

;---------------------------------------------------------------

; Request robot to turn eyes at a specific coordinate point.

(delete-definition "Do gaze at point")
(DefineLink
	(DefinedPredicate "Do gaze at point")
	(LambdaLink
		(VariableList (Variable "$x") (Variable "$y") (Variable "$z"))
		(SequentialAndLink
			(True)
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
		(True)
		))

; -------------------------------------------------------------

; Call once, to fall asleep.
(delete-definition "Do go to sleep")
(DefineLink
	(DefinedPredicate "Do go to sleep")
	(SequentialAnd
		(True)
	))


(delete-definition "Do wake up")
(DefineLink
	(DefinedPredicate "Do wake up")
	(SequentialAnd
		(True)
	))

; -------------------------------------------------------------
; Say something. To test run,
; (cog-evaluate! (Put (DefinedPredicate "Say") (Node "this is a test"))))
(define-public (prt-say-text SENT)
   (format #t "Saying this: ~a\n" SENT)
   (stv 1 1))

(delete-definition "Say")
(DefineLink
	(DefinedPredicate "Say")
	(LambdaLink (Variable "sentence")
		(Evaluation
			(GroundedPredicate "scm: prt-say-text")
			(List (Variable "sentence")))
	))

; -------------------------------------------------------------
; Return true if ROS is still running.
(delete-definition "ROS is running?")
(DefineLink
	(DefinedPredicate "ROS is running?") (True))

; -------------------------------------------------------------
*unspecified*  ; Make the load be silent
