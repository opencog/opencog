;
; movement-api.scm
;
; Definitions providing the movement API.
;
; The definitions here give the API only, but not an implememtation.
; The only current implementation is in the module `(opencog movement)`
; which connects these to ROS blender API robot animations.

(use-modules (opencog logger))

; Delete the current definition, if any.
(define (delete-definition STR)
	(define dfn
		(cog-get-link 'DefineLink 'DefinedPredicateNode
			(DefinedPredicate STR)))

	(if (not (null? dfn)) (cog-delete (car dfn)) #f))

; Printer stub -- Prints to the opencog log file.
(define-public (prt-pred-defn PRED . REST)
   (cog-logger-info "Called (DefinedPredicate \"~a\") with args ~a\n"
		(cog-name PRED) REST)
   (stv 1 1))

; Must print to stdout, so that IRC chatbots can see what happened.
; XXX FIXME -- someday, should probably create a distinct API for
; the IRC text strings.
(define-public (prt-face-expr PRED NAME TIME TENS)
	(format #t "Robot displays facial expression \"~a\" at strength ~a for ~a seconds\n"
		(cog-name NAME)
		(cog-name TIME)
		(cog-name TENS))
	(prt-pred-defn PRED NAME TIME TENS)
)

; As above, but for gestures
(define-public (prt-face-gest PRED NAME TENS RPT SPD)
	(format #t "Robot performs facial gesture \"~a\" at strength ~a speed ~a\n"
		(cog-name NAME)
		(cog-name TENS)
		(cog-name SPD))
	(prt-pred-defn PRED NAME TENS RPT SPD)
)

; As above, but for eye movements
(define-public (prt-gaze-dir PRED X Y Z)
	(format #t "Robot looks at point (~a ~a ~a)\n"
		(cog-name X)
		(cog-name Y)
		(cog-name Z))
	(prt-pred-defn PRED X Y Z)
)

; As above, but for head movements
(define-public (prt-turn-dir PRED X Y Z)
	(format #t "Robot turns head towards (~a ~a ~a)\n"
		(cog-name X)
		(cog-name Y)
		(cog-name Z))
	(prt-pred-defn PRED X Y Z)
)

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
		(EvaluationLink
			(GroundedPredicate "scm: prt-face-expr")
			(ListLink
				(DefinedPredicate "Do show facial expression")
				(Variable "$expr")
				(Variable "$duration")
				(Variable "$intensity"))
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
		(EvaluationLink
			(GroundedPredicate "scm: prt-face-gest")
			(ListLink
				(DefinedPredicate "Do show gesture")
				(Variable "$gest")
				(Variable "$insensity")
				(Variable "$repeat")
				(Variable "$speed"))
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
		(EvaluationLink
			(GroundedPredicate "scm: prt-pred-defn")
			(ListLink
				(DefinedPredicate "Blink rate")
				(Variable "$mean") (Variable "$var"))
		)))

; -------------------------------------------------------------
; Request robot to look at a specific coordinate point.

(delete-definition "Do look at point")
(DefineLink
	(DefinedPredicate "Do look at point")
	(LambdaLink
		(VariableList (Variable "$x") (Variable "$y") (Variable "$z"))
		(EvaluationLink
			(GroundedPredicate "scm: prt-turn-dir")
			(ListLink
				(DefinedPredicate "Do look at point")
				(Variable "$x") (Variable "$y") (Variable "$z"))
		)))

;---------------------------------------------------------------

; Request robot to turn eyes at a specific coordinate point.

(delete-definition "Do gaze at point")
(DefineLink
	(DefinedPredicate "Do gaze at point")
	(LambdaLink
		(VariableList (Variable "$x") (Variable "$y") (Variable "$z"))
		(EvaluationLink
			(GroundedPredicate "scm: prt-gaze-dir")
			(ListLink
				(DefinedPredicate "Do gaze at point")
				(Variable "$x") (Variable "$y") (Variable "$z"))
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
		(EvaluationLink
			(GroundedPredicate "scm: prt-pred-defn")
			(ListLink
				(DefinedPredicate "Publish behavior")
				(Variable "$bhv"))
		)))

; -------------------------------------------------------------

; Call once, to fall asleep.
(delete-definition "Do go to sleep")
(DefineLink
	(DefinedPredicate "Do go to sleep")
	(EvaluationLink
		(GroundedPredicate "scm: prt-pred-defn")
		(ListLink
			(DefinedPredicate "Do go to sleep"))
	))


(delete-definition "Do wake up")
(DefineLink
	(DefinedPredicate "Do wake up")
	(EvaluationLink
		(GroundedPredicate "scm: prt-pred-defn")
		(ListLink
			(DefinedPredicate "Do wake up"))
	))

; -------------------------------------------------------------
; Say something. To test run,
; (cog-evaluate! (Put (DefinedPredicate "Say") (Node "this is a test"))))
(define-public (prt-say-text SENT)
   (cog-logger-info "Saying this: ~a\n" SENT)
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
