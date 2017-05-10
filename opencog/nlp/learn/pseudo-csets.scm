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

(define-public (get-cset-vec ITEM)
"
  get-cset-vec ITEM - Return the vector of pseudo-connector sets
  for the ITEM. ITEM can either be a WordNode, in which case all
  of the csets for that word are returned, or it can be a disjunct,
  in which case all the csets for that disjunct are returned.
"
	; Currently, its as simple as this...
	(cog-incoming-by-type ITEM 'LgWordCset)
)

(define-public (cset-get-word CSET)
"
  Return the word of the CSET
"
	(gar CSET))

(define-public (cset-get-disjunct CSET)
"
  Return the disjunct of the CSET
"
	(gdr CSET))

(define-public (get-word-string WORD)
"
  Get the string rep for the word.
"
	(cog-name WORD))

(define-public (get-disjunct-string DISJUNCT)
"
  Get a string representation for the disjunct
  XXX TODO remove leading blank.
"
	(string-concatenate
		(map
			(lambda (cntor) (string-append " "
					(cog-name (gar cntor)) (cog-name (gdr cntor))))
			(cog-outgoing-set DISJUNCT)))
)

(define-public (get-cset-string CSET)
"
  Get a string representation for the connector-set
"
	(string-append
		(get-word-string (cset-get-word CSET)) ":"
		(get-disjunct-string (cset-get-disjunct CSET)) ";")
)

; Return the cset, if it exists.  If it does not exist, return the
; empty list '()
(define (have-cset? WORD DISJUNCT)
	(cog-link 'LgWordCset WORD DISJUNCT))

; ---------------------------------------------------------------------

(define-public (filter-words-with-csets WORD-LIST)
"
  filter-words-with-csets WORD-LIST - Return the subset of
  the WORD-LIST that has pseudo-csets on them.  This is useful,
  because not all words in the dataset might have csets on them,
  so far.
"
	(filter!
		(lambda (wrd)
			(not (null? (get-cset-vec wrd))))
		WORD-LIST)
)

; ---------------------------------------------------------------------

(define-public (cset-vec-support ITEM)
"
  cset-vec-support ITEM - compute the pseudo-cset vector support for ITEM
  ITEM can be either a WordNode or a disjunct (LgAnd)

  The support of a sparse vector is the number of basis elements that
  are non-zero.  In this case, its simply the number of disjuncts
  attached to the word.  Equivalently, this is the l_0 norm of the
  vector (the l_p norm for p=0).
"
	(length (get-cset-vec ITEM))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-observations ITEM)
"
  cset-vec-observations ITEM - compute the number of observations of
  the disjuncts for ITEM. Equivalently, this is the l_1 norm of the
  vector (the l_p norm for p=1).

  ITEM can be either a WordNode or a disjunct (LgAnd)
"
	; sum of the counts
	(fold
		(lambda (cset sum) (+ (get-count cset) sum))
		0
		(get-cset-vec ITEM))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-len ITEM)
"
  cset-vec-len ITEM - compute the pseudo-cset vector length for ITEM.
  Equivalently, this is the l_2 norm of the vector (the l_p norm for p=2).

  ITEM can be either a WordNode or a disjunct (LgAnd)
"
	; sum of the square of the counts
	(define sumsq
		(fold
			(lambda (cset sum)
				(define cnt (get-count cset))
				(+ sum (* cnt cnt)))
			0
			(get-cset-vec ITEM)))

	(sqrt sumsq)
)

; ---------------------------------------------------------------------

(define-public (cset-vec-lp-norm P ITEM)
"
  cset-vec-lp-norm P ITEM - compute the Banach space l_p norm of
  the pseudo-cset vector for ITEM.

  ITEM can be either a WordNode or a disjunct (LgAnd)
"
	; sum of the powers of the counts
	(define sum
		(fold
			(lambda (cset sum)
				(define cnt (get-count cset))
				(+ sum (expt cnt P)))
			0
			(get-cset-vec ITEM)))

	(expt sum (/ 1 P))
)

; ---------------------------------------------------------------------

(define-public (get-total-cset-count)
"
  get-total-cset-count -- return the total number of observations
  of all connector-sets in the system.
"
	; XXX FIXME, this is somewhat sloppy in it's counting, it
	; does not verify that all atoms that are 'LgWordCsets are
	; valid word+psuedo-disjunct pairs.  Other garbage could
	; sneak in.
	(define tot 0)
	(cog-map-type
		(lambda (cset) (set! tot (+ tot (get-count cset))) #f)
		'LgWordCset)
	tot
)

(define total-cset-count 0)
(define (get-stashed-count)
	; total number of observations of csets in the system.
	; XXX there might be a more elegant way to handle this.
	(if (eqv? 0 total-cset-count)
		(set! total-cset-count (get-total-cset-count)))
	total-cset-count
)

(define-public (get-cset-frequency CSET)
"
  get-cset-frequency -- Return the frequency (probability) with which
  CSET has been observed.
"
	(/ (get-count CSET) (get-stashed-count))
)

(define-public (cset-vec-frequency ITEM)
"
  cset-vec-frequency -- Return the frequency (probability) with which
  ITEM has been observed.
"
	(/ (cset-vec-observations ITEM) (get-stashed-count))
)

(define-public (cset-vec-entropy ITEM)
"
  cset-vec-entropy -- return the entropy for the subset of
  connector-sets associated with ITEM.  ITEM can be either a
  WordNode or a disjunct. A loop is performed over all of the
  csets associated with that item, and the entropy for each
  cset is summed up.

  The returned entropy is in nats. Divide by log 2 to get bits.
"
   ; sum of the counts
   (fold
      (lambda (cset sum)
			(define cset-freq (get-cset-frequency cset))
			(- sum (* cset-freq (log cset-freq))))
      0
      (get-cset-vec ITEM))
)

(define (cset-vec-word-mi WORD)
"
	cset-vec-word-mi - get the fractional mutual information between
   the word and all of it's disjuncts. This is defined as
      MI(w) = (1/p(w)) sum_d p(d,w) log_2 p(d,w)/[p(d,*) p(*,w)]
   This is defined 'fractionally', so that the MI of the dataset
   as a whole can be written as
     MI = sum_w p(w) MI(w)

   Note this function returns MI in units of bits. i.e. log-2
"
	(define n-tot (get-stashed-count))
	(define sum 0)
	(define n-word (cset-vec-observations WORD))
	(for-each
		(lambda (cset)
			(define dj (cset-get-disjunct cset))
			(define n-dj (cset-vec-observations dj))
			(define n-cset (get-count cset))
			(set! sum
				(+ sum (* n-cset (log (/ (* n-cset n-tot) (* n-word n-dj)))))
		))
		(get-cset-vec WORD))
	; (/ sum n-tot)
	(/ sum (* n-word (log 2)))
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------

(define-public (cset-vec-lp-dist P ITEM-A ITEM-B)
"
  cset-vec-lp-dist P ITEM-A ITEM-B - compute the Banache space l_p
  distance between the pseudo-cset vector for ITEM-A and ITEM-B.

  This is a real metric.  Recall, for p=2, this is just the
  Eucliden distance metric, and for p=1, this is the 'Mahnatten
  distance'.

  ITEM can be either a WordNode or a disjunct (LgAnd of pseudo-connectors).

  As a metric for measuring the similarity of connector-set vectors,
  this is terrible, because any word that has a lot of observations
  on it will be a long vector, and thus distant from the origin.
  By contrast, any words that have only a few observations will all
  be near the origin, and thus close to one-another, no matter how
  different thier disjuncts are.
"
	(define word-base (equal? (cog-type ITEM-A) 'WordNode))
	(define get-base (if word-base cset-get-disjunct cset-get-word))

	; Get the common non-zero entries of the two vectors.
	(define bases
		(delete-duplicates! (append!
			(map! get-base (get-cset-vec ITEM-A))
			(map! get-base (get-cset-vec ITEM-B)))))

	; Get the count for DISJUNCT on WORD, if it exists (is non-zero)
	(define (get-cset-count WORD DISJUNCT)
		(define cset (have-cset? WORD DISJUNCT))
		(if (null? cset) 0 (get-count cset))
	)

	(define (cnt-a it)
		(if word-base
			(lambda (it) (get-cset-count ITEM-A it))
			(lambda (it) (get-cset-count it ITEM-A))))

	(define (cnt-b it)
		(if word-base
			(lambda (it) (get-cset-count ITEM-B it))
			(lambda (it) (get-cset-count it ITEM-B))))

	; sum of the powers of the counts
	(define sum
		(fold
			(lambda (it sum)
				(define adiff (abs (- (cnt-a it) (cnt-b it))))
				(+ sum (expt adiff P)))
			0
			bases))

	(expt sum (/ 1 P))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-prod ITEM-A ITEM-B)
"
  cset-vec-prod ITEM-A ITEM-B - compute the pseudo-cset vector
  dot product between ITEM-A and ITEM-B

  ITEMs can be either WordNodes or disjuncts (LgAnd of pseudo-connectors).
"
	(define word-base (equal? (cog-type ITEM-A) 'WordNode))
	(define get-base (if word-base cset-get-disjunct cset-get-word))

	; If the connector-set for ITEM-B exists, then get its count.
	(define (get-other-count cset)
		(define other-cset
			(if word-base
				(have-cset? ITEM-B (cset-get-disjunct cset))
				(have-cset? (cset-get-word cset) ITEM-B)))
		(if (null? other-cset) 0 (get-count other-cset))
	)

	; Loop over all connectors for ITEM-A
	(fold
		(lambda (cset sum)
			(define a-cnt (get-count cset))
			(define b-cnt (get-other-count cset))
			(+ sum (* a-cnt b-cnt)))
		0
		(get-cset-vec ITEM-A))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-cosine ITEM-A ITEM-B)
"
  cset-vec-cosine ITEM-A ITEM-B - compute the pseudo-cset vector
  cosine similarity between ITEM-A and ITEM-B

  ITEMs can be either WordNodes or disjuncts (LgAnd of pseudo-connectors).
"
	(define deno (* (cset-vec-len ITEM-A) (cset-vec-len ITEM-B)))

	(if (eqv? 0.0 deno) 0.0 (/ (cset-vec-prod ITEM-A ITEM-B) deno))
)

; ---------------------------------------------------------------------

(define (get-angle SIM)
	(define pi 3.14159265358979)

	; Stupid-ass guile return a small imaginary number when taking
	; the arccos of 1.0. WTF.  So we need to take he real part!!
	(* 2.0 (/ (real-part (acos SIM)) pi))
)

(define-public (cset-vec-cos-dist ITEM-A ITEM-B)
"
  cset-vec-cos-dist ITEM-A ITEM-B - compute the pseudo-cset vector
  cosine distance between ITEM-A and ITEM-B. The cosine distance
  is defined as dist = 2*arccos(cossim) / pi.  The cosine distance
  obeys the triangle inequality.

  ITEMs can be either WordNodes or disjuncts (LgAnd of pseudo-connectors).
"
	(get-angle (cset-vec-cosine ITEM-A ITEM-B))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-jaccard WORD-A WORD-B)
"
  cset-vec-jaccard WORD-A WORD-B - compute the pseudo-cset vector
  Jaccard distance between WORD-A and WORD-B. The Jaccard distance
  is defined as dist = 1 - sim where sim = sum min(a,b)/ sum max(a,b)

  XXX FIXME this algo is currently written so that it works for
  words only; but it could be made to work for disjuncts, too.
  See the implementation for the lp-distance for the basic outline.
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

; Count number of plus and minus connectors.
; DIR should be (LgConnDirNode "+") or (LgConnDirNode "-")
; Assumes that the direction is held in the second slot...
(define (dj-connectors-dir DISJUNCT DIR)
	(fold
		(lambda (con sum)
			(if (equal? (gdr con) DIR) (+ sum 1) sum))
		0
		(cog-outgoing-set DISJUNCT)))

(define-public (cset-vec-lp-connectors ITEM P)
"
  cset-vec-lp-connectors ITEM - compute the total number of
  observations of the lp-moment of the connectors on the ITEM.
  Specifically, the sum (#connectors)^p.   Note the 1/p root
  is NOT taken!  You probably want to deivce by the number of
  observations first, and then take the 1/p power.
  See cset-vec-connectors for more details.
"
	(define (num-conn cset)
		(dj-connectors (cset-get-disjunct cset)))

	(define (con-count cset)
		(* (get-count cset) (expt (num-conn cset) P)))

	; sum of the counts
	(define sum
		(fold
			(lambda (cset sum) (+ (con-count cset) sum))
			0
			(get-cset-vec ITEM)))

	; (expt sum (/ 1.0 P))
	sum
)

(define-public (cset-vec-connectors ITEM)
"
  cset-vec-connectors ITEM - compute the total number of observations
  of connectors on the ITEM.  Recall that each disjunct is made of
  one or more connectors.  ITEM can be either a word or a disjunct.
  if ITEM is a word, then the total is for all of the observations of
  connectors in disjuncts attached to that word.  If ITEM is a disjunct,
  then the total is for all of the observations of connectors in
  words that contain that disjunct.  This definition is symmetric,
  as counts are stored in cset's, and this merely totals up and weights
  with respect to the items on the other side of ITEM.
"
	(cset-vec-lp-connectors ITEM 1)
)

(define-public (cset-vec-connectors-dir ITEM DIR)
"
  cset-vec-connectors-dir ITEM DIR - compute the total number of
  observations of connectors on the ITEM, but only if the the
  connector goes in direction DIR.  The DIR should be either
  (LgConnDirNode \"+\") or (LgConnDirNode \"-\").

  ITEM can be either a WordNode, or a disjunct (LgAnd).
"
	(define (con-count cset)
		(* (get-count cset) (dj-connectors-dir (cset-get-disjunct cset) DIR)))
	; sum of the counts
	(fold
		(lambda (cset sum) (+ (con-count cset) sum))
		0
		(get-cset-vec ITEM))
)

; ---------------------------------------------------------------------

(define-public (cset-observations ITEM-LIST)
"
  cset-observations ITEM-LIST - Return the number of times that
  all of the connector-sets in the ITEM-LIST have been observed.

  The ITEM-LIST can be a mixture of WordNodes and disjuncts.
"
	(fold
		(lambda (item sum) (+ (cset-vec-observations item) sum))
		0
		ITEM-LIST)
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

(define-public (get-all-csets)
"
  get-all-csets -- Return all of the connector-sets in the atomspace.
  Caution: this performs almost no sanity checks, and so could
  easily return junk, if there are other LgWordCsets's in the atomspace.

  The sanity check would be to make sure that the LgAnd has the desired
  form, i.e. consisting entirely of PseudoDisjuncts.
"
	(define all-csets '())

	(cog-map-type
		(lambda (cset) (set! all-csets (cons cset all-csets)) #f)
		'LgWordCset)

	all-csets
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
