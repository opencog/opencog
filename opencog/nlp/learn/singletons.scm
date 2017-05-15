;
; singletons.scm
;
; Stuff for working with singltons.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; Collection of rancid unfinished junk.

(use-modules (opencog))
(use-modules (opencog persist))

; ---------------------------------------------------------------------
; Count the total number of times that the atoms in the atom-list have
; been observed.  The observation-count for a single atom is stored in
; the 'count' value of its CountTruthValue. This routine just fetches
; those, and adds them up.
;
; The returned value is the total count.

(define-public (get-total-atom-count atom-list)
	;
	; A proceedural loop.
	;	(let ((cnt 0))
	;		(define (inc atom) (set! cnt (+ cnt (get-count atom))))
	;		(for-each inc atom-list)
	;		cnt
	;	)

	; textbook tail-recursive solution.
	(define (hlpr lst cnt)
		(if (null? lst) cnt
			(hlpr (cdr lst) (+ cnt (get-count (car lst))))))

	(hlpr atom-list 0)
)

; ---------------------------------------------------------------------
; Compute the occurance logliklihoods for a list of atoms.
;
; This sums up the occurance-count over the entire list of atoms,
; and uses that as the normalization for the probability frequency
; for the individual atoms in the list. It then computes the log_2
; likelihood for each atom in the list, based on the total.
;
; As usual, the raw counts are obtained from the 'count' slot on a
; CountTruthValue, and the logli is stored as a value on the atom.
;
; This returns the atom-list, but now with the logli's set.

(define (compute-all-logli atom-list)
	(let ((total (get-total-atom-count atom-list)))
		(map
			(lambda (atom) (compute-atom-logli atom total))
			atom-list
		)
	)
)

; ---------------------------------------------------------------------
; Compute the occurance logliklihoods for all words.
;
; Load all word-nodes into the atomspace from SQL storage, if they
; are not already present.  This also loads the associated values.
;
; This returns the list of all word-nodes, with the logli's set.

(define (compute-all-word-freqs)
	(begin
		; Make sure that all word-nodes are in the atom table.
		(call-only-once fetch-all-words)
		(compute-all-logli (get-all-words))
	)
)

; ---------------------------------------------------------------------
