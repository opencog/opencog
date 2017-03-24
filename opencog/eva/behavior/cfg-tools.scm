;
; cfg-tools.scm
;
; Behavior Tree configuration parameters utilities.
;
; Provide handy wrappers that simplify the declaration of configuration
; parameters. Required by `cfg-sophia.scm` and `cfg-eva.scm`.
;
; --------------------------------------------------------
; Emotional-state to expression mapping. For a given emotional state
; (for example, happy, bored, excited), this specifies a range of
; expressions to display for that emotional state, as well as the
; intensities and durations.  `emo-set` adds an expression to an
; emotional state, while `emo-map` is used to set parameters.
(define (emo-expr-set emo-state expression)
	(EvaluationLink
		(PredicateNode "Emotion-expression")
		(ListLink (ConceptNode emo-state) (ConceptNode expression))))

(define (emo-expr-map emo-state expression param value)
	(StateLink (ListLink
		(ConceptNode emo-state) (ConceptNode expression) (SchemaNode param))
		(NumberNode value)))

; Shorthand utility, takes probability, intensity min and max, duration min
; and max.
(define-public (emo-expr-spec emo-state expression
		prob int-min int-max dur-min dur-max)
	(emo-expr-set emo-state expression)
	(emo-expr-map emo-state expression "probability" prob)
	(emo-expr-map emo-state expression "intensity-min" int-min)
	(emo-expr-map emo-state expression "intensity-max" int-max)
	(emo-expr-map emo-state expression "duration-min" dur-min)
	(emo-expr-map emo-state expression "duration-max" dur-max))

; --------------------------------------------------------
; Emotional-state to gesture mapping. For a given emotional state
; (for example, happy, bored, excited), this specifies a range of
; gestures to display for that emotional state, as well as the
; intensities and durations.  `ges-set` adds a gesture to an
; emotional state, while `ges-map` is used to set parameters.
(define (emo-gest-set emo-state gesture)
	(EvaluationLink
		(PredicateNode "Emotion-gesture")
		(ListLink (ConceptNode emo-state) (ConceptNode gesture))))

(define (emo-gest-map emo-state gesture param value)
	(StateLink (ListLink
		(ConceptNode emo-state) (ConceptNode gesture) (SchemaNode param))
		(NumberNode value)))

; Shorthand utility, takes probability, intensity min and max, duration min
; and max, repeat min and max.
(define-public (emo-gest-spec emo-state gesture prob
		int-min int-max rep-min rep-max spd-min spd-max)
	(emo-gest-set emo-state gesture)
	(emo-gest-map emo-state gesture "gest probability" prob)
	(emo-gest-map emo-state gesture "gest intensity-min" int-min)
	(emo-gest-map emo-state gesture "gest intensity-max" int-max)
	(emo-gest-map emo-state gesture "repeat-min" rep-min)
	(emo-gest-map emo-state gesture "repeat-max" rep-max)
	(emo-gest-map emo-state gesture "speed-min" spd-min)
	(emo-gest-map emo-state gesture "speed-max" spd-max))

; --------------------------------------------------------
;
; Dice-roll conditionals. Return true or false, some fraction of the
; time.  The define here sets up an initial value for the probability;
; it can be changed later, at run-time.
;
; The probability is stored in a StateLink; the GetLink performs a
; lookup of the current State value.  The value can be changed simply
; by using the StateLink as normal.
;
; Two strings are auto-generated: the action defnition, by pre-pending
; "dice-roll: " to the action name, and the probability state name,
; by appending " probability" to the action name.
;
; line 588 -- dice_roll("glance_new_face") etc.

(define-public (dice-roll action probability)
	(define prob-name (string-append action " probability"))
	(State (Schema prob-name) (Number probability))
	(DefineLink
		(DefinedPredicateNode (string-append "dice-roll: " action))
		(GreaterThan
			(Get
				(TypedVariable (Variable "$x") (Type "NumberNode"))
				(State (Schema prob-name) (Variable "$x")))
			(RandomNumber (Number 0) (Number 1)))))

; --------------------------------------------------------
