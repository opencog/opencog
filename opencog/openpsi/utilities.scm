; Copyright (C) 2015-2016 OpenCog Foundation
;
; Helper functions for OpenPsi

(use-modules (ice-9 regex)) ; For string-match
(use-modules (srfi srfi-1)) ; For fold, delte-duplicates

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
  psi-get-member-links ATOM - Return list of MemberLinks that hold ATOM.

  All psi rules are members of some ruleset; this searches for and
  finds such MemberLinks.
"
    (define (get-roots an-atom)
        (delete-duplicates (cog-filter 'MemberLink (cog-get-root an-atom))))

    (let ((duals (cog-outgoing-set (cog-execute! (DualLink ATOM)))))
        (if (null? duals)
            (get-roots ATOM)
            (delete-duplicates (concatenate
                (list (append-map get-roots duals)
                    (append (get-roots ATOM)))))
        )
    )
)

; --------------------------------------------------------------
(define-public (psi-get-dual-rules ATOM)
"
  psi-get-dual-rules ATOM - Return list of psi-rules that can ground ATOM.

  ATOM should be a part of a psi-rule.
"
    ; Define a local is-psi-rule? predicate, because the
    ; main one is too slow.
    (define (is-psi-rule? RULE)
        (and
            (equal? 'ImplicationLink (cog-type RULE))
            (equal? 2 (cog-arity RULE))
    ))

    (let ((member-links (psi-get-member-links ATOM)))
         (delete-duplicates (append-map
             (lambda (x) (filter psi-rule? (cog-outgoing-set x)))
             member-links))
    )
)
