(load "../main.scm")
;(use-modules (opencog))
(use-modules (opencog atom-types))  ;needed for AtTimeLink definition?

;;; PARAM DEFINITIONS
;; todo: move to own file

;; todo: should the parmaters be a concept or predicate?
;; todo: and same with the somatic parameters

(define (create-openpsi-param name initial-value)
    (define param
        (Concept (string-append (psi-prefix-str) name) (stv initial-value 1)))
    (Inheritance
        param
        (Concept (string-append (psi-prefix-str) "Parameter")))
    param)

;; todo: make util function to define
(define power (create-openpsi-param "Power" .5))



;;; EVENT PREDICATES
(Predicate "speech-giving starts" (stv 0 1))


;;; SOMATIC PREDICATES
(define somatic-prefix-str "Somatic: ")

(define (create-somatic-predicate name initial-value)
	(define somatic
		(Concept (string-append somatic-prefix-str name) (stv initial-value 1)))
	(Inheritance
		somatic
		(Concept "Somatic"))
	somatic)
;; todo: can a Predicate inherit from a Concept?
;; maybe we instead should do?:
;; (Eval (Predicate "Somatic: voice width") (Predicate "Somatic"))

(define voice-width
	(create-somatic-predicate "voice width" .5))




;;; PARAM UPDATE RULES
;; todo: move to own file

;; When starting a speech -> boost power
;; todo: how to represent "Starts in Interval"? ask Nil

(Implication
	(TypedVariable
		(Variable "$time")
		(Type "TimeNode"))
    (AtTime
        (Variable "$time")
        (Evaluation
	        (Predicate "speech-giving-starts")))
    (AtTime
        (Variable "$time")
        (ExecutionLink
            (DefinedSchema "adjust-openpsi-parameter")
            (List
                power
                (NumberNode .7)))))



; When x changes --> change y
; When power changes -> adjust voice width

(Implication
	(TypedVariable
		(Variable "$time")
		(Type "TimeNode"))
    (AtTime
        (Variable "$time")
        (Evaluation
	        (Predicate "change")
	        (List
	            power)))
    (AtTime
        (Variable "$time")
        (ExecutionLink
            (DefinedSchema "adjust-openpsi-parameter")
            (List
                voice-width
                (NumberNode .3)
                power))))

;; todo: we are not changing the y var in proportion to the change in the x var
;; at this point. We may want to add that.

;(TimeNode (number->string (current-time)))

;; todo:
;; maybe we want some kind of mechanism to indicate a new event is beginning
;; that persists for some specified time duration and then goes away. A recent
;; event pred? Some agent controls/updates this? A context evaluation agent? Is
;; this in the ROS behavioral control loop?

;; check out TriggerLink (some kind of trigger subscription-based messaging system

;; PARAMETER UPDATING

;; alpha is in the range of [-1, 1] and represents the degree of change and
;;     whether the change is positively or negatively correlated with the
;;     origin parameter
;;     0 would lead to no change
;;
;; todo: see case-lambda in scheme for function overloading
(define (adjust-openpsi-parameter target-parm alpha . origin-param)
	(let* ((strength (cog-stv-strength target-parm))
		   (confidence (cog-stv-confidence target-parm))

			; todo: replace this with a function
			(new-strength (min 1 (max 0 (+ strength
				(string->number (cog-name alpha)))))))

		(cog-set-tv! target-parm (cog-new-stv new-strength confidence))
		(display "new strength: ")(display new-strength)(newline)))



