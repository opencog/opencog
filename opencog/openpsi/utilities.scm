; Copyright (C) 2015-2016 OpenCog Foundation
;
; Helper functions for OpenPsi

(use-modules (ice-9 regex)) ; For string-match
(use-modules (srfi srfi-1)) ; For fold, delete-duplicates

(use-modules (opencog) (opencog exec))

; --------------------------------------------------------------
(define-public psi-prefix-str "OpenPsi: ")

; --------------------------------------------------------------
(define-public (psi-suffix-str a-string)
"
  Returns the suffix of that follows `psi-prefix-str` sub-string.

  a-string:
    - a string that should have the`psi-prefix-str`

"
    (let ((z-match (string-match psi-prefix-str a-string)))
        (if z-match
            (match:suffix z-match)
            (error (string-append "The string argument must have the prefix: "
                "\"" psi-prefix-str "\". " "Instead got:" a-string) )
        )
    )
)

; --------------------------------------------------------------
(define-public (satisfaction-level rule)
"
  Returns the probability of satisfaction of the given rule's context as a
  SimpleTruthValue.

  rule:
  - A psi-rule with context to be evaluated.
"
; NOTE
; 1. This is the same as the `psi-satisfiable?`
; 2. Should a context evaluator be added here?????
; 3. What is the "right" way of communicating the level of information.
    (let* ((pattern (SatisfactionLink (AndLink (psi-get-context rule))))
           (result (cog-evaluate! pattern)))
          (cog-delete pattern)
          result
    )
)

; --------------------------------------------------------------
(define-public (most-weighted-atoms atom-list)
"
  It returns a list with non-duplicating atoms with the highest weight. If an
  empty list is passed an empty list is returned. Weight of an atom is the
  product of the stength and confidence of the atom.

  atom-list:
  - A list of atoms to be compared.
"
    (define (weight x)
        (let ((rule-stv (cog-tv x))
              (context-stv (satisfaction-level x)))
            (* (tv-conf rule-stv) (tv-mean rule-stv)
               (tv-conf context-stv) (tv-conf context-stv))))

    (define (pick atom lst) ; prev is a `lst` and next `atom`
        (cond
            ((> (weight (car lst)) (weight atom)) lst)
            ((= (weight (car lst)) (weight atom)) (append lst (list atom)))
            (else (list atom))))

    (if (null? atom-list)
        '()
       (delete-duplicates (fold pick (list (car atom-list)) atom-list))
    )
)

; --------------------------------------------------------------
(define-public (most-important-weighted-atoms atom-list)
"
  It returns a list with non-duplicating atoms with the highest
  important-weight. If an empty list is passed an empty list is returned.
  Weight of an atom is the product of the stength and confidence of the atom.

  atom-list:
  - A list of atoms to be compared.
"
    (define (weight x)
        (let ((a-stv (cog-tv x))
              (sti (assoc-ref (cog-av->alist (cog-av x)) 'sti)))
            (* (tv-conf a-stv) (tv-mean a-stv) sti)))

    (define (pick atom lst) ; prev is a `lst` and next `atom`
        (cond
            ((> (weight (car lst)) (weight atom)) lst)
            ((= (weight (car lst)) (weight atom)) (append lst (list atom)))
            (else (list atom))))

    (if (null? atom-list)
        '()
        (delete-duplicates (fold pick (list (car atom-list)) atom-list))
    )
)

; --------------------------------------------------------------

; Define a local (internal-use-only, thus not define-public) variant
; of the psi-rule? predicate, because the main one is too slow.  This
; checks to see if MEMB is ...
; -- a MemberLink
; -- has arity 2
; -- first elt is an ImplicationLink
; -- Second elt is a node starting with string "OpenPsi: "
;
; Internal-use only, thus, not define-public.
(define (psi-member? MEMB)
    (and
        (equal? 'MemberLink (cog-type MEMB))
        (equal? 2 (cog-arity MEMB))
        (let ((mem (cog-outgoing-set MEMB)))
            (and
                (equal? 'ImplicationLink (cog-type (car mem)))
                (cog-node-type? (cog-type (cadr mem)))
                (string-prefix? psi-prefix-str (cog-name (cadr mem)))
        ))
    ))

; --------------------------------------------------------------

(define-public (psi-get-exact-match ATOM)
"
  psi-get-exact-match ATOM - Return list of all of the MemberLinks
  holding rules whose context or action apply exactly (without
  any variables) to the ATOM. In other words, the ATOM appears
  directly in the context of the rule.

  All psi rules are members of some ruleset; this searches for and
  finds such MemberLinks.
"
    ;; Get all exact matches
    (define inset (cog-get-trunk ATOM))

    ;; Keep only those links that are of type MemberLink...
    ;; and, more precisely, a MmeberLink that is of a valid
    ;; psi-fule form.
    (filter psi-member?
        (delete-duplicates (cog-filter 'MemberLink inset)))
)

(define-public (psi-get-dual-match ATOM)
"
  psi-get-dual-match ATOM - Return list of the MemberLinks
  holding rules whose context or action might apply to ATOM,
  as a generalized case (i.e. containining variables).

  All psi rules are members of some ruleset; this searches for and
  finds such MemberLinks.
"
    (define set-of-duals (cog-execute! (DualLink ATOM)))

    ;; Get all patterned rules
    (define duset
        (concatenate
            (map cog-get-trunk (cog-outgoing-set set-of-duals))))

    ; Avoid garbaging up the atomspace.
    (cog-delete set-of-duals)

    ;; Keep only those links that are of type MemberLink...
    ;; and, more precisely, a MmeberLink that is of a valid
    ;; psi-fule form.
    (filter psi-member?
        (delete-duplicates (cog-filter 'MemberLink duset)))
)

(define-public (psi-get-members ATOM)
"
  psi-get-members ATOM - Return list of all of the MemberLinks
  holding rules whose context or action might apply to ATOM.

  All psi rules are members of some ruleset; this searches for and
  finds such MemberLinks.
"
    (delete-duplicates (concatenate! (list
        (psi-get-exact-match ATOM)
        (psi-get-dual-match ATOM)
    )))
)

; --------------------------------------------------------------
(define (functionality-pattern tag-node functionlity functionality-name)
"
  Returns a StateLink with the following structure
    (StateLink
      (ListLink
          (Node (string-append psi-prefix-str functionality-name))
           tag-node)
       functionlity)

  tag-node:
  - A demand/modulator node that the functionality is being added to.

  functionlity:
  - A DefinedPredicateNode/DefinedSchemaNode that is evaluated for performing
    the functionality over the particular demand/modulator.

  functionality-name:
  - The type of functionality.
"
    (StateLink
        (ListLink
            (Node (string-append psi-prefix-str functionality-name))
             tag-node)
         functionlity)
)

; --------------------------------------------------------------
(define-public
    (psi-set-functionality functionlity is-eval tag-node functionality-name)
"
  This function is used to add a functionality to a particular demand/modulator.

  functionlity:
  - A DefinedPredicateNode/DefinedSchemaNode that is evaluated for performing
    the functionality over the particular demand/modulator.

  is-eval:
  - #t if the functionality is evaluatable and #f if not.

  tag-node:
  - A demand/modulator node that the functionality is being added to.

  functionality-name:
  - The type of functionality.
"
    (define (check-alias a-name)
        (if is-eval
            (cog-node 'DefinedPredicateNode a-name)
            (cog-node 'DefinedSchemaNode a-name)))

    (let* ((name (string-append
                        psi-prefix-str functionality-name "-"
                        (cog-name tag-node)))
           (alias (check-alias name)))

       (if (null? alias)
           (begin
               (set! alias
                    (if is-eval
                        (DefinedPredicateNode name)
                        (DefinedSchemaNode name)
                    )
                )
               (DefineLink alias functionlity)
               (functionality-pattern tag-node alias functionality-name)
                alias
           )
            alias ; The assumption is that the EvaluationLink is already created
       )
    )
)

; --------------------------------------------------------------
(define-public (psi-get-functionality tag-node functionality-name)
"
  Returns a list with the node that represents the functionality for the given
  demand/modulator or nil if it doesn't exist.

  tag-node:
  - A demand/modulator node that the functionality is being added to.

  functionality-name:
  - The type of functionality.
"
; The assumption is that there will be only one element in the returned list.
; This is a weak. Need a better way of using DefineLink short of defining
; the relationship in the DefinedSchema/Predicate as a part of the alias-node
; name.
    (cog-outgoing-set (cog-execute! (GetLink
        (functionality-pattern tag-node (Variable "$x") functionality-name))))
)

; --------------------------------------------------------------
; --------------------------------------------------------------
; Getters and setters for openpsi-related entity/parameter values
;
; Presently the implementation assumes the values of openpsi parameters are
; normalized in [0 1]. The current values of the params are assumed to be stored
; in a StateLink or if not in a StateLink an attempt is made to evaluate or
; execute the atom to obtain a value.

(define (psi-get-value entity)
"
  Get the current value of a psi-related entity. For entities with numerical
  values, and NumberNode is returned.
"
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
"
	Get the current value of psi-related entity and return as a number (rather
	than NumberNode).
"
	(define result (psi-get-value entity))
	;(format #t "psi-get-number-value entity: \n~a initial result: ~a\n"
	;	entity result)
	(if (and (cog-atom? result) (eq? 'NumberNode (cog-type result)))
		(set! result (string->number (cog-name result))))
	(if (cog-tv? result)
		(set! result (tv-mean result)))
    ;(format #t "return result: ~a\n" result)
	result)

(define (psi-set-value! entity value)
"
  Set the current value of psi-related entity.
"
	; OpenPsi values are stored using StateLinks by default
	; Todo: need to handle case where value is not stored in StateLink
	(State
		entity
		value))
