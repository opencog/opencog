;
; cfg-tools.scm
;
; Behavior Tree configuration parameters utilities.
;
; Provide handy wrappers that simplify the declaration of configuration
; parameters. Required by behavior-cfg.scm
;
; --------------------------------------------------------
; Emotional-state to expression mapping. For a given emotional state
; (for example, happy, bored, excited) this specifies a range of
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
(define (emo-expr-spec emo-state expression
		prob int-min int-max dur-min dur-max)
	(emo-expr-set emo-state expression)
	(emo-expr-map emo-state expression "probability" prob)
	(emo-expr-map emo-state expression "intensity-min" int-min)
	(emo-expr-map emo-state expression "intensity-max" int-max)
	(emo-expr-map emo-state expression "duration-min" dur-min)
	(emo-expr-map emo-state expression "duration-max" dur-max))

; --------------------------------------------------------
; Emotional-state to gesture mapping. For a given emotional state
; (for example, happy, bored, excited) this specifies a range of
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
(define (emo-gest-spec emo-state gesture prob
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
