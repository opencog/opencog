;;; DYNAMIC INTERACTION UPDATE RULES

;(define psi-interaction-rule-str
;	(string-append psi-prefix-str "interaction rule"))
(define psi-interaction-rule
	(ConceptNode (string-append psi-prefix-str "interaction rule")))

(define (create-psi-interaction-rule antecedent consequent strength)
	(define rule
		(PredictiveImplication
			(TimeNode 1)
		    (Evaluation
                ;(GroundedPredicate "scm: psi-change-in?")
                (Predicate "psi-changed")
                (List
                    antecedent))
		    (ExecutionOutputLink
	            (GroundedSchema "scm: adjust-psi-var-level")
	            (List
	                consequent
	                (NumberNode strength)
	                antecedent))))
	(Member rule psi-interaction-rule)
	rule
)


; --------------------------------------------------------------
; The Rules

(define speech->power
	(create-psi-interaction-rule speech agent-state-power .5))

(define power->voice
	(create-psi-interaction-rule agent-state-power voice-width .5))

(define power->arousal
	(create-psi-interaction-rule agent-state-power arousal .5))

(define arousal->voice
	(create-psi-interaction-rule arousal voice-width .5))

