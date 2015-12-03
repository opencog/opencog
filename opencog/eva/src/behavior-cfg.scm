;
; behavior-cfg.scm
;
; Behavior Tree configuration parameters.
; Under construction.
;
; This is (meant to be) a verbatim translation of the contents of the
; old `behavior.cfg` file.  However, a more elegant design should now be
; possible, so this is ... mae not he best way to do things.
;

; default_emotion_duration aka current_emotion_duration
; (StateLink (SchemaNode "current_emotion_duration") (TimeNode 1.0)) ; in seconds

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

; Translation of behavior.cfg line 9 ff
(emo-expr-spec "new-arrival" "surprised"  1.0 0.2 0.4 10 15)

(emo-expr-spec "frustrated" "sad"         0.4 0.6 0.8 5 15)
(emo-expr-spec "frustrated" "confused"    0.1 0.4 0.6 5 15)
(emo-expr-spec "frustrated" "recoil"      0.2 0.1 0.2 5 15)
(emo-expr-spec "frustrated" "surprised"   0.2 0.1 0.2 5 15)

(emo-expr-spec "positive" "happy"         0.2 0.6 0.8 10 15)
(emo-expr-spec "positive" "comprehending" 0.3 0.5 0.8 10 15)
(emo-expr-spec "positive" "engaged"       0.4 0.5 0.8 10 15)

(emo-expr-spec "bored"    "bored"         0.7 0.4 0.7 10 15)
(emo-expr-spec "bored"    "sad"           0.1 0.1 0.3 10 15)
(emo-expr-spec "bored"    "happy"         0.2 0.1 0.3 10 15)

(emo-expr-spec "sleep"    "happy"         1.0  0.0 0.1 5 15)

(emo-expr-spec "wake-up"  "surprised"     0.45 0.2 0.6 5 15)
(emo-expr-spec "wake-up"  "happy"         0.2  0.5 0.7 5 15)
(emo-expr-spec "wake-up"  "irritated"     0.6  0.1 0.4 1  4)

(emo-expr-spec "neutral-speech"  "happy"         0.2  0.1 0.3 4 8)
(emo-expr-spec "neutral-speech"  "comprehending" 0.4  0.5 0.8 4 8)
(emo-expr-spec "neutral-speech"  "engaged"       0.4  0.5 0.8 4 8)

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
	(emo-gest-map emo-state gesture "probability" prob)
	(emo-gest-map emo-state gesture "intensity-min" int-min)
	(emo-gest-map emo-state gesture "intensity-max" int-max)
	(emo-gest-map emo-state gesture "repeat-min" rep-min)
	(emo-gest-map emo-state gesture "repeat-max" rep-max)
	(emo-gest-map emo-state gesture "speed-min" spd-min)
	(emo-gest-map emo-state gesture "speed-max" spd-max))

; Translation of behavior.cfg line 75 ff
(emo-gest-spec "positive" "nod-1"  0.1 0.6 0.9 1 1 0.5 0.8)
(emo-gest-spec "positive" "nod-2"  0.1 0.2 0.4 1 1 0.8 0.9)

(emo-gest-spec "bored"   "yawn-1"  0.01 0.6 0.9 1 1 1 1)

(emo-gest-spec "sleep"  "blink-sleepy"  1 0.7 1.0 1 1 1 1)

(emo-gest-spec "wake-up" "shake-2"  0.4 0.7 1.0 1 1 0.7 0.8)
(emo-gest-spec "wake-up" "shake-3"  0.3 0.6 1.0 1 1 0.7 0.8)
(emo-gest-spec "wake-up" "blink"    0.3 0.8 1.0 2 4 0.9 1.0)

(emo-gest-spec "listening" "think-browsUp.001"  0.4 0.7 1.0 1 1 0.6 0.8)
(emo-gest-spec "listening" "think-browsUp.003"  0.3 0.6 1.0 1 1 0.6 0.8)
(emo-gest-spec "listening" "think-L.up"         0.3 0.8 1.0 1 1 0.6 1.0)

; --------------------------------------------------------
