;
; pseudo-csets.scm
;
; Compute the cosine distance between two pseuo-connector-sets.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below compute the cosine-distance between pseudo
; connector-set vectors.
;
; An example connector-set, for the word "playing", illustrating
; that it can connect to the word "level" on the left, and "field"
; on the right, is shown below. In terms of ENglish grammar, it is
; a bad example, because English should not connect this way. But
; it is an example...
;
;    (LgWordCset
;       (WordNode "playing")
;       (LgAnd
;          (PseudoConnector
;             (WordNode "level")
;             (LgConnDirNode "-"))
;          (PseudoConnector
;             (WordNode "field")
;             (LgConnDirNode "+"))))
;
; Any given word may have dozens or hundreds or thousands of these
; connector sets. The totality of these sets, for a given, fixed word
; form a vector.  The `LgAnd` is a basis element, and the raw
; observational count on the `LgWordCset` is the magnitude of the
; of the vector in that basis direction.
;
; Note that these vectors are sparse: if a particular `LgAnd` is
; missing, then the associated count is zero.  Note that the dimension
; of the vector-space is extremely high: its strictly larger than the
; number of observed word-pairs, which is tens-of-millions.
;
; As vectors, dot-products can be taken. The most interesting of these
; is the cosine distance between two words. This distance indicates how
; similar two words are, grammatically-speaking. 
;
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (opencog))
(use-modules (opencog persist))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------

(define-public (fetch-pseudo-connectors WORD-LIST)
"
  fetch-pseudo-cpnnectors WORD-LIST - fetch (from the database)
  all pseudo-connectors for all of the WordNodes in the WORD-LIST.
"
	(define (fetch-one WORD)
		(fetch-incoming-by-type WORD 'LgWordCset))

	(define start-time (current-time))
	; (for-each fetch-one WORD-LIST)
	(load-atoms-of-type 'LgWordCset)
	(format #t "Elapsed time to load words: ~A secs\n"
		(- (current-time) start-time))
)

; ---------------------------------------------------------------------

(define-public (filter-words-with-connectors WORD-LIST)
"
  filter-words-with-connectors WORD-LIST - Return the subset of
  the WORD-LIST that has connectors
"
	(filter
		(lambda (wrd)
			(not (null? (cog-incoming-by-type wrd 'LgWordCset))))
		WORD-LIST)
)

; ---------------------------------------------------------------------

(define-public (cset-vec-support WORD)
"
  cset-vec-support WORD - compute the pseudo-cset vector support for WORD
  The support of a sparse vector is the number of basis elements that
  are non-zero.
"
	(length (cog-incoming-by-type WORD 'LgWordCset))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-len WORD)
"
  cset-vec-len WORD - compute the pseudo-cset vector length for WORD
"
	; sum of the square of the counts
	(define sumsq
		(fold
			(lambda (cset sum)
				(define cnt (get-count cset))
				(+ sum (* cnt cnt)))
			0
		(cog-incoming-by-type WORD 'LgWordCset)))

	(sqrt sumsq)
)

; ---------------------------------------------------------------------

(define-public (cset-vec-prod WORD-A WORD-B)
"
  cset-vec-prod WORD-A WORD-B - compute the pseudo-cset vector
  dot product between WORD-A and WORD-B
"
	; If the connector-set for WORD-B exists, then get its count.
	(define (get-cset-count LGAND)
		(define cset (cog-link 'LgWordCset WORD-B LGAND))
		(if (null? cset) 0 (get-count cset))
	)

	; Loop over all connectors for WORD-A
	(fold
		(lambda (cset sum)
			(define a-cnt (get-count cset))
			(define b-cnt (get-cset-count (gdr cset)))
			(+ sum (* a-cnt b-cnt)))
		0
		(cog-incoming-by-type WORD-A 'LgWordCset))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-cosine WORD-A WORD-B)
"
  cset-vec-cosine WORD-A WORD-B - compute the pseudo-cset vector
  cosine distance between WORD-A and WORD-B
"
	(/ (cset-vec-prod WORD-A WORD-B)
		(* (cset-vec-len WORD-A) (cset-vec-len WORD-B)))
)

; ---------------------------------------------------------------------
