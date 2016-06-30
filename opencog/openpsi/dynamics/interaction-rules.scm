;
; interaction-rules.scm
;
; Rules specfying the dynamic interactions between different OpenPsi related
; entities.
;
; They express rules of the form:
;  "change in trigger-entity results in change in target-entity with strength s"
;
; The change in the target is a function of the magnitude of change in the
; trigger, the strength of the interaction rule, and the current value of the
; target.
;
; The atomese form of the rules is:
; (PredictiveImplication
;  			(TimeNode 1)
;  		    (Evaluation
;                  (Predicate "psi-changed")
;                  (List
;                      trigger))
;  		    (ExecutionOutputLink
;  	            (GroundedSchema "scm: adjust-psi-var-level")
;  	            (List
;  	                target
;  	                (NumberNode strength)
;  	                trigger))))


(define psi-interaction-rule
	(ConceptNode (string-append psi-prefix-str "interaction rule")))

(define (create-psi-interaction-rule trigger-entity target-entity strength)
"
  Helper function to create the interaction rules. It is assumed that the
  trigger and target store their current values in a StateLink, and if not are
  evaluatable or executable with the result being the current value. If the
  entity is an evaluatable predicate, current value of the entity is assumed to
  be the strength of its evaluated truth value.

  trigger-entity - the variable that when it changes triggers a change in target
  target-entity - the variable that is changed as a result of change in trigger
"
	(define rule
		(PredictiveImplication
			(TimeNode 1)
		    (Evaluation
                (Predicate "psi-changed")
                (List
                    trigger-entity))
		    (ExecutionOutputLink
	            (GroundedSchema "scm: adjust-psi-var-level")
	            (List
	                target-entity
	                (NumberNode strength)
	                trigger-entity))))
	(Member rule psi-interaction-rule)
	rule
)


; --------------------------------------------------------------
; The Rules

(define speech->power
	(create-psi-interaction-rule speech agent-state-power .5))

(define power->voice
	(create-psi-interaction-rule agent-state-power voice-width 1))

(define power->arousal
	(create-psi-interaction-rule agent-state-power arousal .1))

(define arousal->voice
	(create-psi-interaction-rule arousal voice-width -.9))

