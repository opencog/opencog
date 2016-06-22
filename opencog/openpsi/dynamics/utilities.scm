; Utilities.scm

; Utilities for openpsi internal interaction dyanmics

; --------------------------------------------------------------
; Getters and setters for openpsi entity values

(define (psi-get-value entity)
	(define result #f)

	; First check for StateLink value
	; OpenPsi values are stored using StateLinks
	(define resultset
		(cog-execute!
				(Get
					(State
						entity
						(Variable "$n")))))
	(if (not (null? (cog-outgoing-set resultset)))
		(set! result (gar resultset))

		; else check if entity is an evaluation or predicate or schema
		(let ((type (cog-type entity)))
			(if (or (equal? type 'PredicateNode)
				   (equal? type 'GroundedPredicateNode)
				   (equal? type 'DefinedPredicateNode))
				(set! result (cog-evaluate! (Evaluation entity (List)))))
			(if (or (equal? type 'SchemaNode)
            		(equal? type 'GroundedSchemaNode)
            	    (equal? type 'DefinedSchemaeNode))
            	(set! result (cog-execute! (ExecutionOutput entity (List)))))
            (if (equal? type 'EvaluationLink)
                (set! result (cog-evaluate! entity)))
        )
	)
	result
)

(define (psi-get-number-value entity)
	(define result (psi-get-value entity))
	(if (and (cog-atom? result) (eq? 'NumberNode (cog-type result)))
		(set! result (string->number (cog-name result))))
	(if (cog-tv? result)
		(set! result (tv-mean result)))
	result)

(define (psi-set-value! entity value)
	; OpenPsi values are stored using StateLinks
	(State
		entity
		value))
