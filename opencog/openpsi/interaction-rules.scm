;
; interaction-rules.scm
;
; Rules specfying the dynamic interactions between different OpenPsi related
; entities.
;
; They express rules of the logical form:
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

; Contains defined entities to be used in the interaction rules
(load "entity-defs.scm")

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
  strength - the degree to which a change in the trigger event causes a change
    in the target. Value is in [0,1]
"
	(define rule)
	; Todo: Why is this executing the error when the file is loaded?
	;(if (or (< strength 0) (> strength 1))
    ;    (error (string-append "In function create-psi-interaction rule, "
    ;        "strength parameter needs to be in [0,1]")))
	(set! rule
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
;
; Todo: Rule sets should be moved to their own files.

; ==============================================================
; Below are bogus mock interaction rules for dev purposes

(define speech->power
	(create-psi-interaction-rule speech agent-state-power .5))

(define power->voice
	(create-psi-interaction-rule agent-state-power voice-width 1))

(define power->arousal
	(create-psi-interaction-rule agent-state-power arousal .1))

(define arousal->voice
	(create-psi-interaction-rule arousal voice-width -.9))
; ==============================================================

