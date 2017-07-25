;
; interaction-rule.scm
;
; Copyright 2016 OpenCog Foundation
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
;
; The valid predicates for the antecdent are:
;
;       (Predicate "psi-changed") - trigger entity value has changed
;       (Predicate "psi-increased") - trigger entity value has increased
;       (Predicate "psi-decreased") - trigger entity value has decreased
;
; To create an interaction rule, use the psi-create-interaction-rule function,
; which has documentation with the function.
; e.g., (psi-create-interaction-rule positive-sentiment-dialog
;        		increased pos-valence .3))

(use-modules (opencog atom-types)) ; needed for PredictiveImplication definition

; Define the change-predicate types that can be used in the interaction rules.
(define changed "changed")
(define increased "increased")
(define decreased "decreased")

(define psi-interaction-rule
	(ConceptNode (string-append psi-prefix-str "interaction rule")))

(define (psi-create-interaction-rule trigger-entity change-type target-entity
	strength)
"
  Helper function to create the interaction rules. It is assumed that the
  trigger and target store their current values in a StateLink by default, and
  if not are evaluatable or executable with the result being the current value.
  If the entity is an evaluatable predicate, current value of the entity is
  assumed to be the strength of its evaluated truth value.

  Parameters:
  trigger-entity - the variable that when it changes triggers a change in target
  change-type - specifies type of change in the trigger.
				options: changed, increased, decreased
  target-entity - the variable that is changed as a result of change in trigger
  strength - the degree to which a change in the trigger event causes a change
	in the target.
	Value is in [-1,1].
	Positive value means target changes in the same direction as the trigger
	Negative value means target changes in the opposite direction as the trigger
	Large magnitude means change in trigger has a large effect on target value
	Small magnitude means change in trigger has small effect on target value
	0 would mean no change occurs in the target, rendering such a rule useless
"
	(define rule)
	(define pred-change-type-name
		(cond
			((equal? change-type changed) "psi-changed")
			((equal? change-type increased) "psi-increased")
			((equal? change-type decreased) "psi-decreased")
			(else "error")))
	(if (eq? pred-change-type-name "error")
		(error (string-append "In function create-psi-interaction rule, "
			"change-type parameter needs to be one of 'changed', 'increased' "
			"or 'decreased'. got: " change-type)))
	(if (or (< strength -1) (> strength 1))
		(error (string-append "In function create-psi-interaction rule, "
			"strength parameter needs to be in [-1,1]. got: "
			(number->string strength))))

	(set! rule
		(PredictiveImplication
			(TimeNode 1)
			(Evaluation
				(Predicate pred-change-type-name)
				(List
					trigger-entity))
			(ExecutionOutputLink
				(GroundedSchema "scm: adjust-psi-var-level")
				(List
					target-entity
					(NumberNode strength)
					trigger-entity))))
	(Member rule psi-interaction-rule)
	;(format #t (string-append "\npsi-create-interaction-rule  trigger: ~a  type:"
	;	"  ~a  strength: ~a  target: ~a\nrule: ~a") trigger-entity change-type
	;	strength target-entity rule)
	rule
)


(define (psi-create-general-rule antecedent consequent consequent-params)
	(define rule
		(PredictiveImplication
			(TimeNode 1)
			antecedent
			(ExecutionOutputLink
				consequent
				consequent-params)))
	(Member rule psi-interaction-rule)
	;(display rule)
	rule
)
