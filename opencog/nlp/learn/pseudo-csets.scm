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
; The `LgAnd` part of this structure is refered to as the
; pseudo-disjunct, in that it resembles a normal linkgrammar
; disjunct, but has word appearing where connectors should be.
;
; Any given word may have dozens or hundreds or thousands of these
; connector sets. The totality of these sets, for a given, fixed word
; form a vector.  The disjunct is a basis element, and the raw
; observational count on the `LgWordCset` is the magnitude of the
; of the vector in that basis direction.
;
; Note that these vectors are sparse: if a particular disjunct is
; missing, then the associated count is zero.  Note that the dimension
; of the vector-space is extremely high, possibly in the
; tens-of-millions.
;
; As vectors, dot-products can be taken. The most interesting of these
; is the cosine distance between two words. This distance indicates how
; similar two words are, grammatically-speaking. Other vector measures
; are interesting, including lp-products, the Tanimoto metric, the
; Otsuka-Ochiai coefficient, and so on.
;
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (opencog))
(use-modules (opencog persist))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------

(define-public (fetch-pseudo-csets WORD-LIST)
"
  fetch-pseudo-csets WORD-LIST - fetch (from the database)
  all pseudo-csets for all of the WordNodes in the WORD-LIST.
"
	(define (fetch-one WORD)
		(fetch-incoming-by-type WORD 'LgWordCset))

	(define start-time (current-time))
	; (for-each fetch-one WORD-LIST) ; this is wyyyyy too slow!
	(load-atoms-of-type 'LgWordCset)
	(format #t "Elapsed time to load words: ~A secs\n"
		(- (current-time) start-time))
)

; ---------------------------------------------------------------------

(define-public (filter-words-with-csets WORD-LIST)
"
  filter-words-with-csets WORD-LIST - Return the subset of
  the WORD-LIST that has pseudo-csets on them.
"
	(filter!
		(lambda (wrd)
			(not (null? (cog-incoming-by-type wrd 'LgWordCset))))
		WORD-LIST)
)

; ---------------------------------------------------------------------

(define-public (cset-vec-support WORD)
"
  cset-vec-support WORD - compute the pseudo-cset vector support for WORD
  The support of a sparse vector is the number of basis elements that
  are non-zero.  In this case, its simply the number of disjuncts
  attached to the word.  Equivalently, this is the l_0 norm of the
  vector (the l_p norm for p=0).
"
	(length (cog-incoming-by-type WORD 'LgWordCset))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-observations WORD)
"
  cset-vec-len WORD - compute the number of observations of
  the disjuncts for WORD. Equivalently, this is the l_1 norm of the
  vector (the l_p norm for p=1).
"
	; sum of the counts
	(fold
		(lambda (cset sum) (+ (get-count cset) sum))
		0
	(cog-incoming-by-type WORD 'LgWordCset))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-len WORD)
"
  cset-vec-len WORD - compute the pseudo-cset vector length for WORD.
  Equivalently, this is the l_2 norm of the vector (the l_p norm for p=2).
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

(define-public (cset-vec-lp-norm P WORD)
"
  cset-vec-lp-norm P WORD - compute the l_p norm of the pseudo-cset
  vector for WORD.
"
	; sum of the powers of the counts
	(define sum
		(fold
			(lambda (cset sum)
				(define cnt (get-count cset))
				(+ sum (expt cnt P)))
			0
		(cog-incoming-by-type WORD 'LgWordCset)))

	(expt sum (/ 1 P))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-prod WORD-A WORD-B)
"
  cset-vec-prod WORD-A WORD-B - compute the pseudo-cset vector
  dot product between WORD-A and WORD-B
"
	; If the connector-set for WORD-B exists, then get its count.
	(define (get-cset-count DISJUNCT)
		(define cset (cog-link 'LgWordCset WORD-B DISJUNCT))
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
	(define deno (* (cset-vec-len WORD-A) (cset-vec-len WORD-B)))

	(if (eqv? 0.0 deno) 0.0 (/ (cset-vec-prod WORD-A WORD-B) deno))
)

; ---------------------------------------------------------------------

(define-public (cset-observations WORD-LIST)
"
  cset-observations WORD-LIST - Return the number of times that
  all of the disjuncts in the WORD-LIST have been observed.
"
	(fold
		(lambda (word sum) (+ (cset-vec-observations word) sum))
		0
		WORD-LIST)
)

; ---------------------------------------------------------------------

(define-public (cset-support WORD-LIST)
"
  cset-support WORD-LIST - Return all of the disjuncts in use
  in the space of all cset-vectors in the WORD-LIST.  Equivalently,
  return all of the basis vectors in the space.  One disjunct is
  just one basis element.

  Caution: this can take a very long time!
"
	(define all-csets
		(append-map!
			(lambda (wrd) (cog-incoming-by-type wrd 'LgWordCset))
			WORD-LIST))

	(delete-duplicates!
		(map!
			(lambda (cset) (gdr cset))
			all-csets))
)

; ---------------------------------------------------------------------

(define-public (cset-dimension WORD-LIST)
"
  cset-dimension WORD-LIST - Return the dimension (size of support)
  of the space ov all cset-vectors in the WORD-LIST.

  Caution: this can take a very long time!
"
	(length (cset-support))
)

; ---------------------------------------------------------------------
