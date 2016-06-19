; Copyright (C) 2015-2016 OpenCog Foundation
;
; Helper functions for OpenPsi

(use-modules (ice-9 regex)) ; For string-match
(use-modules (srfi srfi-1)) ; For fold, delete-duplicates

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
(define (functionality-pattern tag-node functionality-name value-node)
    (StateLink
        (ListLink
            (Node (string-append psi-prefix-str functionality-name))
             tag-node)
         value-node)
)

; --------------------------------------------------------------
(define-public (psi-set-functionality term is-eval tag-node functionality-name)
"
  is-eval: #t if it is evaluatable and #f if not.
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
               (DefineLink alias term)
               (functionality-pattern tag-node functionality-name alias)
                alias
           )
            alias ; The assumption is that the EvaluationLink is already created
       )
    )
)

; --------------------------------------------------------------
(define (psi-get-functionality tag-node functionality-name)
; The assumption is that there will be only one element in the returned list.
; This is a weak. Need a better way of using DefineLink short of defining
; the relationship in the predicatenode as a part of the alias-node name.
    (cog-outgoing-set (cog-execute! (GetLink
        (functionality-pattern tag-node functionality-name (Variable "$x")))))
)
