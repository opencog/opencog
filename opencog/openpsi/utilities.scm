;
; utilities.scm
; Helper functions for OpenPsi
;
; Copyright (C) 2015-2016 OpenCog Foundation

(use-modules (ice-9 regex)) ; For string-match
(use-modules (srfi srfi-1)) ; For fold, delete-duplicates

(use-modules (opencog) (opencog exec))

; --------------------------------------------------------------
; XXX TODO: does this really need to be public?
(define psi-prefix-str "OpenPsi: ")

; --------------------------------------------------------------
; XXX TODO: does this really need to be public?
(define (psi-suffix-str a-string)
"
  psi-suffix-str STRING

  Given the string STRING, this removes the psi prefix string.
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

(define (psi-get-exact-match ATOM)
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
    ;; and, more precisely, a MemberLink that is of a valid
    ;; psi-fule form.
    (filter psi-member?
        (delete-duplicates (cog-filter 'MemberLink inset)))
)

(define (psi-get-dual-match ATOM)
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
    ;; and, more precisely, a MemberLink that is of a valid
    ;; psi-fule form.
    (filter psi-member?
        (delete-duplicates (cog-filter 'MemberLink duset)))
)

(define (psi-get-members ATOM)
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
(define
    (psi-set-functionality functionlity is-eval tag-node functionality-name)
"
  psi-set-functionality FUNC IS-EVAL TAG FUNC-NAME

  Associate a function with a particular demand or modulator.

  FUNC is an atom that can be executed or evaluated. It will perform
    the functionality for the particular demand/modulator.

  Set IS-EVAL to #t if the functionality is evaluatable and #f if
    it is executable.

  TAG should be a demand or modulator node that the functionality will
    be assocaited with.

  FUNC-NAME is the type of functionality.
"
    ;; XXX FIXME -- there is no need to force the use of DPN's or DSN's
    ;; here. Any excutable or evaluatable atom should be allowed.
    (define (check-alias a-name)
        (if is-eval
            (cog-node 'DefinedPredicateNode a-name)
            (cog-node 'DefinedSchemaNode a-name)))

    (let* ( (name (string-append
                        psi-prefix-str functionality-name "-"
                        (cog-name tag-node)))
            (alias (check-alias name)))

        (if (null? alias)
            (begin
                (set! alias
                     (if is-eval
                         (DefinedPredicateNode name)
                         (DefinedSchemaNode name)))

                ;; XXX FIXME why do we need a DefineLink here???
                ;; why is an alias needed? what is the point of this?
                (DefineLink alias functionlity)
                (StateLink
                    (ListLink
                        (Node (string-append psi-prefix-str functionality-name))
                         tag-node)
                     alias)
                alias
            )
        )
        alias
    )
)

; --------------------------------------------------------------
(define (psi-get-functionality tag-node functionality-name)
"
  psi-get-functionality TAG FUNC-NAME

  Return a list with the node that represents the functionality for the given
  demand/modulator or nil if it doesn't exist.

  TAG should be a demand/modulator node that the functionality is
  being added to.

  FUNC-NAME should be the type of functionality.
"
    (define state
       (ListLink
           (Node (string-append psi-prefix-str functionality-name))
           tag-node))

    (cog-outgoing-set (cog-execute!
        (GetLink (StateLink state (Variable "$x")))))
)

; --------------------------------------------------------------
