;
; express.scm
;
; Generate facial expressions and gestures, based on the "personality
; configuration" paramaters in the `behavior-cfg.scm` file.
;
; Currently, this just picks out random expressions and gestures
; from a given expression/gesture class: for example, the "positive"
; expressions are "happy", "comprehending" and "engaged"; the random
; selector will pick one of these, according to the weights in the
; config file.
;
; It calls into the action orchestrator to display the expressions.
; (not implemented yet).
;
; --------------------------------------------------------
; Given the name of an emotional-expression class, randomly pick one
; of the facial expression animations in that class, at random.
;
; Example usage:
;
;   (cog-execute!
;      (PutLink (DefinedSchemaNode "Pick random expression")
;         (ConceptNode "positive")))
;
; This will pick out one of the "positive" expressions (defined in the
; previously-loaded `cfg-*.scm` file) and return it, as a `Concept`.
; For example: `(ConceptNode "comprehending")`
;
(DefineLink
	(DefinedSchema "Pick random expression")
	(LambdaLink
		(Variable "$emo")
		(RandomChoice
			(GetLink
				; Return a bunch of probability-expr pairs.
				(VariableList
					(TypedVariable (Variable "$prob") (Type "NumberNode"))
					(TypedVariable (Variable "$expr") (Type "ConceptNode")))
				(AndLink
					(Evaluation
						(Predicate "Emotion-expression")
						(ListLink (Variable "$emo") (Variable "$expr")))
					(State
						(ListLink
							(Variable "$emo")
							(Variable "$expr")
							(Schema "probability"))
						(Variable "$prob"))
				)))))

; As above, but for gestures.
(DefineLink
	(DefinedSchema "Pick random gesture")
	(LambdaLink
		(Variable "$emo")
		(RandomChoice
			(GetLink
				(VariableList
					(TypedVariable (Variable "$prob") (Type "NumberNode"))
					(TypedVariable (Variable "$expr") (Type "ConceptNode")))
				(AndLink
					(Evaluation
						(Predicate "Emotion-gesture")
						(ListLink (Variable "$emo") (Variable "$expr")))
					(State
						(ListLink
							(Variable "$emo")
							(Variable "$expr")
							(Schema "gest probability"))
						(Variable "$prob"))
				)))))

; Pick a random numeric value, lying in the range between min and max.
; The range min and max depends on a emotion-class+facial-expression
; pair. For an example usage, see below.
(define (pick-value-in-range min-name max-name)
	(LambdaLink
		(VariableList (VariableNode "$emo") (VariableNode "$expr"))
		(RandomNumberLink
			(GetLink
				(TypedVariable (Variable "$int-min") (Type "NumberNode"))
				(StateLink (ListLink
					(VariableNode "$emo") (VariableNode "$expr")
					(SchemaNode min-name)) (VariableNode "$int-min")))
			(GetLink
				(TypedVariable (Variable "$int-max") (Type "NumberNode"))
				(StateLink (ListLink
					(VariableNode "$emo") (VariableNode "$expr")
					(SchemaNode max-name)) (VariableNode "$int-max")))
		)))

; Get a random intensity value for the indicated emotion-class +
; facial-expression. That is, given an emotion-class+facial-expression
; pair, this wil look up the min and max allowed intensity levels, and
; return a random number between these min and max values.
;
; Example usage:
;    (cog-execute!
;        (PutLink (DefinedSchemaNode "get random intensity")
;            (ListLink (ConceptNode "positive") (ConceptNode "engaged"))))
;
; will return an intensity level for the `positive-engaged` expression.
; It returns a `NumberNode`.
(DefineLink
	(DefinedSchemaNode "get random intensity")
	(pick-value-in-range "intensity-min" "intensity-max"))

; As above, but for gestures. Avoids a name-space collision.
(DefineLink
	(DefinedSchemaNode "get random gest intensity")
	(pick-value-in-range "gest intensity-min" "gest intensity-max"))

; Similar to above, but for duration. See explanation above.
(DefineLink
	(DefinedSchemaNode "get random duration")
	(pick-value-in-range "duration-min" "duration-max"))

(DefineLink
	(DefinedSchemaNode "get random repeat")
	(pick-value-in-range "repeat-min" "repeat-max"))

(DefineLink
	(DefinedSchemaNode "get random speed")
	(pick-value-in-range "speed-min" "speed-max"))

; Show a facial expression from a given emotional class. Sends the
; expression to the action orchestrator for display.  The intensity
; and duration of the expression is picked randomly from the configuration
; parameters for the emotion class.
;
; Example usage:
;    (cog-evaluate!
;        (PutLink (DefinedPredicateNode "Show class expression")
;           (ListLink (ConceptNode "positive") (ConceptNode "engaged"))))
;
(DefineLink
	(DefinedPredicateNode "Show class expression")
	(LambdaLink
		(VariableList (VariableNode "$emo") (VariableNode "$expr"))
		;; Send it off to the action-orchestrator to actually do it.
		(PutLink (DefinedPredicate "Show facial expression")
			(ListLink
				(VariableNode "$expr")
				(PutLink
					(DefinedSchemaNode "get random duration")
					(ListLink (VariableNode "$emo") (VariableNode "$expr")))
				(PutLink
					(DefinedSchemaNode "get random intensity")
					(ListLink (VariableNode "$emo") (VariableNode "$expr")))
		))
	))

; Show a gesture for a given emotional-expression class. Sends the
; gesture to ROS for display.  The intensity, repetition and speed of the
; gesture is picked randomly from the parameters for the facial-gesture.
;
; Example usage:
;    (cog-evaluate!
;        (PutLink (DefinedPredicateNode "Show class gesture")
;           (ListLink (ConceptNode "positive") (ConceptNode "nod-1"))))
;
(DefineLink
	(DefinedPredicateNode "Show class gesture")
	(LambdaLink
		(VariableList (VariableNode "$emo") (VariableNode "$gest"))
		;; Send it off to the action orchestrator to actually do it.
		(PutLink (DefinedPredicate "Show gesture")
			(ListLink
				(VariableNode "$gest")
				(PutLink
					(DefinedSchemaNode "get random gest intensity")
					(ListLink (VariableNode "$emo") (VariableNode "$gest")))
				(PutLink
					(DefinedSchemaNode "get random repeat")
					(ListLink (VariableNode "$emo") (VariableNode "$gest")))
				(PutLink
					(DefinedSchemaNode "get random speed")
					(ListLink (VariableNode "$emo") (VariableNode "$gest")))
		))
	))

;
; Pick a facial expression out of the given emotional-expression class,
; and send it to the action orchestrator for display.
; The expression is picked randomly from the class of expressions for
; the given emotion.  Likewise, the strength of display, and the
; duration are picked randomly.
;
; Example usage:
;    (cog-evaluate!
;       (PutLink (DefinedPredicateNode "Show random expression")
;          (ConceptNode "positive")))
;
; The above will pick one of the "positive" facial expressions, and
; send it off to ROS.
;
(DefineLink
	(DefinedPredicateNode "Show random expression")
	(LambdaLink
		(VariableNode "$emo")
		(PutLink
			(DefinedPredicateNode "Show class expression")
			(ListLink
				(VariableNode "$emo")
				(PutLink
					(DefinedSchemaNode "Pick random expression")
					(VariableNode "$emo"))
			))
	))

;; Like the above, but for gestures
(DefineLink
	(DefinedPredicate "Show random gesture")
	(LambdaLink
		(Variable "$emo")
		(Put
			(DefinedPredicate "Show class gesture")
			(ListLink
				(Variable "$emo")
				(Put
					(DefinedSchema "Pick random gesture")
					(Variable "$emo"))
			))
	))

; --------------------------------------------------------
; Show facial expressions and gestures suitable for a given emotional
; state. These are random selectors, picking some expression randomly
; from a menu of choices, and displaying it.

;; Pick random expression, and display it.
(DefineLink
	(DefinedPredicateNode "Show positive expression")
	(PutLink (DefinedPredicateNode "Show random expression")
		(ConceptNode "positive")))

;; line 840 -- show_frustrated_expression()
(DefineLink
	(DefinedPredicateNode "Show frustrated expression")
	(PutLink (DefinedPredicateNode "Show random expression")
		(ConceptNode "frustrated")))

;; Pick random positive gesture
(DefineLink
	(DefinedPredicateNode "Pick random positive gesture")
	(PutLink (DefinedPredicateNode "Show random gesture")
		(ConceptNode "positive")))

;; ------------------------------------------------------------------
