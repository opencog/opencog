;
; pseudo-csets.scm
;
; Compute the cosine-similarity between two pseudo-connector-sets.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below compute the cosine-similarity (and other similarity
; meaasures) between pseudo-connector-set vectors.
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
; is the cosine similarity between two words. This quantity indicates how
; similar two words are, grammatically-speaking. Other vector measures
; are interesting, including lp-similarity, etc.
;
; Not implemented: the Pearson R.  There is both a theoretical and a
; practical difficulty with it. The theoretical difficulty is that
; wen never expect connector sets to be correlated, with have a
; different mean offset!  This doesn't make sense, because when a
; disjunct is not seen, it literally isn't there; it does NOT get
; folded into a mean-value offset.  The practical difficulty is that
; computing the vector mean requires knowning the total dimensionality
; of the space, which can be gotten by computing
;   (length (cset-support get-all-words))
; but this can take hours to compute.  Its also huge: about 200K
; for my "small" dataset; millions in the large one.
;
; Also not implemented: Tanimoto metric. The formula for this gives
; an actual metric only when the vectors are bit-vectors, and we don't
; have bit-vectors, so I am not implementing this.
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

(define-public (get-cset-vec WORD)
"
  get-cset-vec WORD - Return the vector of pseudo-connector sets
  for the WORD. WORD can either be a WordNode, in which case all
  of the csets for that word are returned, or it can be a disjunct,
  in which case all the csets for that disjunct are returned.
"
	; Currently, its as simple as this...
	(cog-incoming-by-type WORD 'LgWordCset)
)

; Return the word of the CSET
(define (cset-get-word CSET) (gar CSET))

; Return the disjunct of the CSET
(define (cset-get-disjunct CSET) (gdr CSET))

; Return the cset, if it exists.  If it does not exist, return the
; empty list '()
(define (have-cset? WORD DISJUNCT)
	(cog-link 'LgWordCset WORD DISJUNCT))

; ---------------------------------------------------------------------

(define-public (filter-words-with-csets WORD-LIST)
"
  filter-words-with-csets WORD-LIST - Return the subset of
  the WORD-LIST that has pseudo-csets on them.
"
	(filter!
		(lambda (wrd)
			(not (null? (get-cset-vec wrd))))
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
	(length (get-cset-vec WORD))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-observations WORD)
"
  cset-vec-observations WORD - compute the number of observations of
  the disjuncts for WORD. Equivalently, this is the l_1 norm of the
  vector (the l_p norm for p=1).
"
	; sum of the counts
	(fold
		(lambda (cset sum) (+ (get-count cset) sum))
		0
		(get-cset-vec WORD))
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
			(get-cset-vec WORD)))

	(sqrt sumsq)
)

; ---------------------------------------------------------------------

(define-public (cset-vec-lp-norm P WORD)
"
  cset-vec-lp-norm P WORD - compute the Banach space l_p norm of
  the pseudo-cset vector for WORD.
"
	; sum of the powers of the counts
	(define sum
		(fold
			(lambda (cset sum)
				(define cnt (get-count cset))
				(+ sum (expt cnt P)))
			0
			(get-cset-vec WORD)))

	(expt sum (/ 1 P))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-lp-dist P WORD-A WORD-B)
"
  cset-vec-lp-dist P WORD-A WORD-B - compute the Banache space l_p
  distance between the pseudo-cset vector for WORD-A and WORD-B.

  This is a real metric.  Recall, for p=2, this is just the
  Eucliden distance metric, and for p=1, this is the 'Mahnatten
  distance'.

  As a metric for measuring the similarity of connector-set vectors,
  this is terrible, because any word that has a lot of observations
  on it will be a long vector, and thus distant from the origin.
  By contrast, any words that have only a few observations will all
  be near the origin, and thus close to one-another, no matter how
  different thier disjuncts are.
"
	; Get the common non-zero entries of the two vectors.
	(define disjuncts (delete-duplicates! (append!
		(map! cset-get-disjunct (get-cset-vec WORD-A))
		(map! cset-get-disjunct (get-cset-vec WORD-B)))))

	; Get the count for DISJUNCT on WORD, if it exists (is non-zero)
	(define (get-cset-count WORD DISJUNCT)
		(define cset (have-cset? WORD DISJUNCT))
		(if (null? cset) 0 (get-count cset))
	)

	; sum of the powers of the counts
	(define sum
		(fold
			(lambda (dj sum)
				(define cnt-a (get-cset-count WORD-A dj))
				(define cnt-b (get-cset-count WORD-B dj))
				(define adiff (abs (- cnt-a cnt-b)))
				(+ sum (expt adiff P)))
			0
			disjuncts))

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
		(define cset (have-cset? WORD-B DISJUNCT))
		(if (null? cset) 0 (get-count cset))
	)

	; Loop over all connectors for WORD-A
	(fold
		(lambda (cset sum)
			(define a-cnt (get-count cset))
			(define b-cnt (get-cset-count (cset-get-disjunct cset)))
			(+ sum (* a-cnt b-cnt)))
		0
		(get-cset-vec WORD-A))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-cosine WORD-A WORD-B)
"
  cset-vec-cosine WORD-A WORD-B - compute the pseudo-cset vector
  cosine similarity between WORD-A and WORD-B
"
	(define deno (* (cset-vec-len WORD-A) (cset-vec-len WORD-B)))

	(if (eqv? 0.0 deno) 0.0 (/ (cset-vec-prod WORD-A WORD-B) deno))
)

; ---------------------------------------------------------------------

(define (get-angle SIM)
	(define pi 3.14159265358979)

	; Stupid-ass guile return a small imaginary number when taking
	; the arccos of 1.0. WTF.  So we need to take he real part!!
	(* 2.0 (/ (real-part (acos SIM)) pi))
)

(define-public (cset-vec-cos-dist WORD-A WORD-B)
"
  cset-vec-cos-dist WORD-A WORD-B - compute the pseudo-cset vector
  cosine distance between WORD-A and WORD-B. The cosine distance
  is defined as dist = 2*arccos(cossim) / pi.  The cosine distance
  obeys the triangle inequality.
"
	(get-angle (cset-vec-cosine WORD-A WORD-B))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-jaccard WORD-A WORD-B)
"
  cset-vec-jaccard WORD-A WORD-B - compute the pseudo-cset vector
  Jaccard distance between WORD-A and WORD-B. The Jaccard distance
  is defined as dist = 1 - sim where sim = sum min(a,b)/ sum max(a,b)
"
	; Get the common non-zero entries of the two vectors.
	(define disjuncts (delete-duplicates! (append!
		(map! cset-get-disjunct (get-cset-vec WORD-A))
		(map! cset-get-disjunct (get-cset-vec WORD-B)))))

	; Get the count for DISJUNCT on WORD, if it exists (is non-zero)
	(define (get-cset-count WORD DISJUNCT)
		(define cset (have-cset? WORD DISJUNCT))
		(if (null? cset) 0 (get-count cset))
	)

	; Sum over the min of all pairs
	(define sum-inf 0)
	; Sum over the max of all pairs
	(define sum-sup 0)

	(for-each
		(lambda (dj)
			(define cnt-a (get-cset-count WORD-A dj))
			(define cnt-b (get-cset-count WORD-B dj))
			(set! sum-inf (+ sum-inf (min cnt-a cnt-b)))
			(set! sum-sup (+ sum-sup (max cnt-a cnt-b))))
		disjuncts)

	(- 1.0 (/ sum-inf sum-sup))
)

; ---------------------------------------------------------------------
; Count the number of connectors in the disjunct.
; But the disjunct is just an LgAnd of a bunch of connectors,
; so this is easy.
(define (dj-connectors DISJUNCT) (length (cog-outgoing-set DISJUNCT)))

(define-public (cset-vec-connectors WORD)
"
  cset-vec-connectors WORD - compute the total number of
  observations of connectors on the word.  Recall that each disjunct
  is made of one or more connectors.
"
	(define (con-count cset)
		(* (get-count cset) (dj-connectors (cset-get-disjunct cset))))
	; sum of the counts
	(fold
		(lambda (cset sum) (+ (con-count cset) sum))
		0
		(get-cset-vec WORD))
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
  just one basis element.  The length of this list is the total
  dimension of the space.

  Caution: this can take a very long time!  Last time, this took
  3-4 hours to return a list of length 200K for 430K observations
  distributed across 30K words.

  For practical use, use `(get-all-disjuncts)` below.
"
	(define all-csets
		(append-map!
			(lambda (wrd) (get-cset-vec wrd))
			WORD-LIST))

	(delete-duplicates!
		(map!
			(lambda (cset) (cset-get-disjunct cset))
			all-csets))
)

; ---------------------------------------------------------------------

(define-public (get-all-disjuncts)
"
  get-all-disjuncts -- Return all of the disjuncts in the atomspace.
  Caution: this performs almost no sanity checks, and so could
  easily return junk, if there are other LgAnd's in the atomspace.

  The sanity check would be to make sure that the LgAnd has the desired
  form, i.e. consisting entirely of PseudoDisjuncts.
"
	(define all-djs '())

	(cog-map-type
		(lambda (dj) (set! all-djs (cons dj all-djs)) #f)
		'LgAnd)

	all-djs
)

; ---------------------------------------------------------------------
; Example usage:
;
; (use-modules (opencog) (opencog persist) (opencog persist-sql))
; (use-modules (opencog nlp) (opencog nlp learn))
; (sql-open "postgres:///en_pairs_mst?user=linas")
; (fetch-all-words)
; (length (get-all-words))
; 396262
; (fetch-pseudo-csets (get-all-words))
; (define ac (filter-words-with-csets (get-all-words)))
; (length ac)
; 30127  now 37413
; (define ad (get-all-disjuncts))
; (length ad)
; 200183 now 291637
;
; (cset-vec-cosine (Word "this") (Word "that"))
; (cset-vec-cosine (Word "he") (Word "she"))
