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
; -------------------------------------------------------------
; Request a display of a facial expression (smile, frown, etc.)
; The expression name should be one of the supported blender animations.
;
; Example usage:
;    (cog-evaluate! (Put (DefinedPredicate "Show expression")
;         (ListLink (Concept "happy") (Number 6) (Number 0.3))))
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
			(TrueLink (DefinedSchemaNode "set expression timestamp"))
			;; Send it off to ROS to actually do it.
			(EvaluationLink (GroundedPredicateNode "py:do_emotion")
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
			(EvaluationLink (GroundedPredicateNode "py:do_gesture")
				(ListLink
					(Variable "$gest")
					(Variable "$insensity")
					(Variable "$repeat")
					(Variable "$speed")))
		)))

x
