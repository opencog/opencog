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
(define-public (most-weighted-atoms atom-list)
"
  It returns a list with non-duplicating atoms with the highest weight. If an
  empty list is passed an empty list is returned. Weight of an atom is the
  product of the stength and confidence of the atom.

  atom-list:
  - A list of atoms to be compared.
"
    (define (weight x)
        (let ((a-stv (cog-tv x)))
            (* (tv-conf a-stv) (tv-mean a-stv))))

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

(define-public (psi-get-member-links ATOM)
"
  psi-get-member-links ATOM - Return list of all of the MemberLinks
  holding rules whose context or action may apply to ATOM.

  All psi rules are members of some ruleset; this searches for and
  finds such MemberLinks.
"
    (define set-of-duals (cog-execute! (DualLink ATOM)))

    (define inset '())

    ;; Recursively get all links that contain the given atom.
    ;; Append them to the list "inset"
    (define (get-iset atom)
        (define iset (cog-incoming-set atom))
        (if (not (null? iset))
             (begin
                 (set! inset (concatenate! (list inset iset)))
                 (for-each get-iset iset))
        )
    )

    (get-iset ATOM)
    (for-each get-iset (cog-outgoing-set set-of-duals))

    ; Avoid garbaging up the atomspace.
    (cog-delete set-of-duals)

    ;; Keep only those links that are of type MemberLink
    (delete-duplicates (cog-filter 'MemberLink inset))
)

; --------------------------------------------------------------
(define-public (psi-get-dual-rules ATOM)
"
  psi-get-dual-rules ATOM - Return list of psi-rules that can ground ATOM.

  ATOM should be a part of a psi-rule.
"
    ; Define a local variant of the psi-rule? predicate, because the
    ; main one is too slow.  This checks to see if MEMB is ...
    ; -- a MemberLink
    ; -- has arity 2
    ; -- first elt is an ImplicationLink
    ; -- Second elt is a node starting with string "OpenPsi: "
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

    (let ((member-links (psi-get-member-links ATOM)))
         (delete-duplicates (append-map
             (lambda (x) (filter psi-member? (list x)))
             member-links))
    )
)
