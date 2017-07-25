; Copyright (C) 2015-2017 OpenCog Foundation

(use-modules (opencog) (opencog exec))

; --------------------------------------------------------------
; Getters and setters for openpsi-related entity/parameter values
;
; Presently the implementation assumes the values of openpsi parameters are
; normalized in [0 1]. The current values of the params are assumed to be stored
; in a StateLink or if not in a StateLink an attempt is made to evaluate or
; execute the atom to obtain a value.

(define (cog-get-state-value entity)
	(define query
		(Get
			(State
				entity
				(Variable "$n"))))
	(define result (cog-execute! query))
	;(cog-delete query) ; maybe more optimal to keep this in the atomspace
	(if (not (null? (cog-outgoing-set result)))
		(gar result)
		#f))


(define (cog-set-state-value entity value)
			(State
				entity
				value))


(define (psi-get-value entity)
"
  Get the current value of a psi-related entity. For entities with numerical
  values a NumberNode is returned.
"
	;(define result #f)

	; Todo: Could potential optimize? here by using
	; psi-value-representation-type fucntion

	; First check for StateLink value
	(define result (cog-get-state-value entity))
	(if result
		; if result is not #f return it
		result
		; else check if entity is an evaluation or predicate or schema
		(let ((type (cog-type entity)))
			(if (equal? type 'GroundedPredicateNode)
				(set! result (cog-evaluate! (Evaluation entity (List)))))
			(if  (equal? type 'DefinedPredicateNode)
				(set! result (cog-evaluate! entity)))
			(if (equal? type 'PredicateNode)
				(set! result (cog-tv entity)))
			(if (or (equal? type 'GroundedSchemaNode)
                    (equal? type 'DefinedSchemaNode))
            	(set! result (cog-execute! (ExecutionOutput entity (List)))))
            (if (equal? type 'EvaluationLink)
                (set! result (cog-evaluate! entity)))
        )
	)
	result
)

(define (psi-get-number-value entity)
"
	Get the current value of psi-related entity and return as a number (rather
	than NumberNode).
	Todo: How to handle non-number returns. #f?
"
	(define result (psi-get-value entity))
	;(format #t "psi-get-number-value entity: \n~a initial result: ~a\n"
	;	entity result)
	(if (and (cog-atom? result) (eq? 'NumberNode (cog-type result)))
		(set! result (string->number (cog-name result))))
	; if result is a tv and confidenct is 0, means that it has not been set
	(if (and (cog-tv? result) (> (tv-conf result) 0))
		(set! result (tv-mean result)))
	(if (not (number? result))
		(set! result #f))
    ;(format #t "return result: ~a\n" result)
	result)


; OpenPsi entity current value representation types
(define statelink "StateLink")
(define evaluatable "Evaluatable")
(define executable "Executable")
(define evaluationlink "EvaluationLink") ; not sure if we need this yet
(define executionlink "ExecutionLink") ; ditto
(define undefined "Undefined")

(define (psi-set-value! entity value)
"
  Set the current numerical value of psi-related entity.

  entity - the object whose value is being set
  value - a numerical value (not NumberNode), assumed to be in [0,1] (for now)
"
	(define value-rep-type (psi-value-representation-type entity))
	(define representation) ; the atomese representation of the stored value

	;(format #t "\npsi-set-value! \n  entity: ~a  value: ~a  value-rep-type: ~a\n"
    ;		entity value value-rep-type)

	(set! representation
		(cond
			((equal? value-rep-type statelink) (State entity (Number value)))
			((equal? value-rep-type evaluatable)
			    (if (equal? (cog-type entity) 'PredicateNode)
			        ; If PredicateNode, then set node TV
			        (cog-set-tv! entity (stv value 1))
			        ; else wrap it in an EvaluationLink
			        ; Todo: this will probably need to be changed to handle
			        ; arguments.
				    (Evaluation entity (List) (stv value 1))))
			((equal? value-rep-type executable)
				(ExecutionOutput entity (List) (stv value 1)))
			(else (error (string-append "In psi-set-value! encountered undefined"
			    " value representation type: ") value-rep-type))))

	;(format #t "representation: ~a" representation)
)

(define psi-rep-type-node (Concept "value-representation-type"))

(define (psi-value-representation-type entity)
"
  Returns the representation type used to store the current value of entity.
  Potential return values: 'StateLink' 'Evaluatable' 'Executable'
  'Evaluation and ExecutionLink themselves?'

  Value representation type is stored in a StateLink:
        State
            List
                entity
                Concept 'value-representation-type'
            Concept '<the type for this entity>'
"
	;(define rep-type undefined)

	(define (set-value-rep-type! entity type)
		(State
            (List
                entity
                psi-rep-type-node)
            (Concept type)))

	; first see if representation type is already set for this entity
	(define rep-type (cog-get-state-value (List entity psi-rep-type-node)))

	;(format #t (string-append "\npsi-value-representation-type \n  entity: ~a  "
	;	"initial rep-type: ~a\n") entity rep-type)

	(if rep-type
		; return the stored representation type
		(cog-name rep-type)

		; else value representation type is not yet set
		;begin
		; Check if value is stored in StateLink, which is the default
		(if (eq? (tv-mean
			(cog-evaluate!
				(Satisfaction
					(State
						entity
						(Variable "$n"))))) 1)
			(begin
				(set-value-rep-type! entity statelink)
	            ;(format #t (string-append "Found value stored in SateLink. "
	            ;    "Setting value-rep type to ~a\n") statelink)
				;(set! rep-type statelink)
				; let's see if it will return from here
				statelink)

			; else check if entity is a predicate or schema
			(let ((atom-type (cog-type entity)))
				(if (or (equal? atom-type 'PredicateNode)
					   (equal? atom-type 'GroundedPredicateNode)
					   (equal? atom-type 'DefinedPredicateNode))
					(let ((confidence (tv-conf
							(cog-evaluate! (Evaluation entity (List))))))
						(if (not (eq? confidence 0))
							(begin
								(set-value-rep-type! entity evaluatable)
								(set! rep-type evaluatable)))))
				(if (or (equal? atom-type 'SchemaNode)
	                    (equal? atom-type 'GroundedSchemaNode)
	                    (equal? atom-type 'DefinedSchemaeNode))
	                (begin
	                    (set-value-rep-type! entity executable)
	                    (set! rep-type executable)))

	            ; Not sure if we need this for EvaluationLinks and
	            ; ExecutionLinks.
	            ;(if (equal? atom-type 'EvaluationLink)
	            ;    (begin (set-rep-type... ) (set! rep-type ...)))

	            ; If no current value is set for the entity, then set the value
	            ; type to statelink, which is the default
	            (if (not rep-type)
	                (begin
		                (set-value-rep-type! entity statelink)
		                (set! rep-type statelink)))
	            ;(format #t "Set value-rep type: ~a\n" rep-type)
	            rep-type))))

(define (psi-get-number-values-for-vars . vars)
"
	Get the current numerical values for a list of psi-related internal
	variables. This function is written for calling via the REST API interface,
	so the list of variables consist of string values of the variable names
	(rather than the variables themselves).

	vars - psi-related variable names as string values (rest arguments)

	Returns a JSON representation of key-value pairs in the form of
	{var_name: value, var_name2: value, ... }
	If a psi variable with varname is not defined, #f is returned for the value.
"
	(define return '())

	; Internal function to add the variable value of varname to the return list
	(define (append-var-value varname)
		(define value)
		(define var-node (Concept (string-append psi-prefix-str varname)))
		(set! value (psi-get-number-value var-node))
		; If value is not set (iow, equal to #f), set it to null for javascript
		; compatibility.
		(if (eq? value #f)
			(set! value "null"))
		(set! return
			(append return (list (format #f "\"~a\": ~a" varname value))))
	)

	(for-each append-var-value vars)

	; return a string JSON object
	(string-append "{" (string-join return ", ") "}")
)

; --------------------------------------------------------------
; Baseline value functionality
(define psi-baseline-value-node (Concept "psi-baseline-value"))

(define (psi-set-baseline-value! modulator value)
  (psi-set-value!
    (List modulator psi-baseline-value-node) value))

(define (psi-get-baseline-value modulator)
  (psi-get-number-value
    (List modulator psi-baseline-value-node)))
