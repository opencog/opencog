;
; word-cosines.scm
;
; Compute the cosine-similarity between two pseudo-connector-sets.
; These provide convenience wrappers on the pseudo-connector-set api.
; These were used by some of the graphing scripts that were used to
; create graphs for the diary.  If/when the graphings scripts are
; modernized, much of the contents of this file can be removed.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below compute the cosine-similarity (and other similarity
; measures) between pseudo-connector-set vectors.  They're deprecated;
; they are mostly one-liner wrappers around the proper matrix API.
;
; See the file `psuedo-csets.scm` for a technical explanation of what
; the vector is.
;
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (opencog))
(use-modules (opencog persist))
(use-modules (opencog matrix))

(define pseudo-cset-api (make-pseudo-cset-api))
(define pseudo-cset-count-api (add-pair-count-api pseudo-cset-api))
(define pseudo-cset-freq-api (add-pair-freq-api pseudo-cset-api))
(define pseudo-cset-support-api (add-support-api pseudo-cset-api))
(define pseudo-cset-cosine-api (add-pair-cosine-compute pseudo-cset-api))
(define pseudo-cset-stars (add-pair-stars pseudo-cset-api))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------

(define-public (get-cset-vec ITEM)
"
  get-cset-vec ITEM - Return the vector of pseudo-connector sets
  for the ITEM. ITEM can either be a WordNode, in which case all
  of the csets for that word are returned, or it can be a disjunct,
  in which case all the csets for that disjunct are returned.
"
	; Currently, its as simple as this...
	(cog-incoming-by-type ITEM 'Section)

	; Should be this: XXX FIXME later
	; (pseudo-cset-support-api 'right-stars ITEM)
)

(define-public (sort-cset-vec ITEM)
"
  sort-cset-vec ITEM - Return the vector of pseudo-connector sets
  for the ITEM, in sorted order of the number of observations.
"
	(sort (get-cset-vec ITEM)
		(lambda (a b)
			(> (get-count a) (get-count b))))
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
	(cog-link 'Section WORD DISJUNCT))

(define-public (print-cset-list PORT LIST)
"
  print-cset-list PORT LIST -- print a list of connector sets to PORT.

  The list is pretty-printed, with the number of observations,
  the word, and the disjuncts.
"
	(for-each
		(lambda (cset)
			(format PORT "~A	~A	~A\n"
				(get-count cset)
				(cog-name (cset-get-word cset))
				(get-disjunct-string (cset-get-disjunct cset))))
		LIST)
)

; ---------------------------------------------------------------------

(define-public (get-all-disjuncts)
"
  get-all-disjuncts -- Return all of the disjuncts in the atomspace.
"
	(pseudo-cset-stars 'right-basis)
)

; ---------------------------------------------------------------------

(define-public (get-all-cset-words)
"
  get-all-cset-words -- Return all of the words that appear in some
  connector set.
"
	(pseudo-cset-stars 'left-basis)
)

; ---------------------------------------------------------------------

(define-public (cset-vec-word-support WORD)
"
  cset-vec-word-support WORD - return size of the support for WORD.
  WORD must be a WordNode.

  The support of a sparse vector is the number of basis elements that
  are non-zero.  In this case, its simply the number of disjuncts
  attached to the word.  Equivalently, this is the l_0 norm of the
  vector (the l_p norm for p=0).
"
	; Should be same as
	; (length (pseudo-cset-support-api 'right-support-set WORD))
	(pseudo-cset-support-api 'right-support WORD)
)

; ---------------------------------------------------------------------

(define-public (cset-vec-word-observations WORD)
"
  cset-vec-word-observations WORD - return the number of times that
  WORD has been observed.  This is exactly equal to to wild-card
  summation over disjuncts N(w) = N(w,*) = sum_d N(w,d) for w being
  the WordNode WORD.  This is exactly the same as the l_1 norm of
  of the vector (the l_p norm for p=1).

  WORD must be WordNode.
"
	; Should be same as
	; (pseudo-cset-support-api 'right-count WORD)
	(pseudo-cset-count-api 'right-wild-count WORD)
)

; ---------------------------------------------------------------------

(define-public (cset-vec-dj-observations DISJUNCT)
"
  cset-vec-dj-observations DISJUNCT - return the number of times that
  DISJUNCT has been observed.  This is exactly equal to to wild-card
  summation over words N(*,d) = sum_w N(w,d) for d being the DISJUNCT.

  DISJUNCT must be an ConnectorSeq.
"
	; Should be same as
	; (pseudo-cset-support-api 'left-count DISJUNCT)
	(pseudo-cset-count-api 'left-wild-count DISJUNCT)
)

; ---------------------------------------------------------------------

(define-public (cset-vec-word-len WORD)
"
  cset-vec-word-len WORD - compute the pseudo-cset vector length for WORD.
  Equivalently, this is the l_2 norm of the vector (the l_p norm for p=2).

  WORD must be a WordNode.
"
	; square-root of the sum of the square of the counts
	(pseudo-cset-support-api 'right-length WORD)
)

; ---------------------------------------------------------------------

(define-public (get-total-cset-count)
"
  get-total-cset-count -- return the total number of observations
  of all connector-sets in the system.
"
	(pseudo-cset-count-api 'wild-wild-count)
)

(define-public (get-cset-frequency CSET)
"
  get-cset-frequency -- Return the frequency (probability) with which
  CSET has been observed. CSET must be a link of type Section.
"
	(pseudo-cset-freq-api 'pair-freq CSET)
)

(define-public (cset-vec-word-frequency WORD)
"
  cset-vec-word-frequency -- Return the frequency (probability) with
  which WORD has been observed. WORD must be a WordNode.
"
	(pseudo-cset-freq-api 'right-wild-freq WORD)
)

(define-public (cset-vec-dj-frequency DISJUNCT)
"
  cset-vec-dj-frequency -- Return the frequency (probability) with
  which DISJUNCT has been observed. DISJUNCT must be an ConnectorSeq.
"
	(pseudo-cset-freq-api 'left-wild-freq DISJUNCT)
)

(define-public (cset-vec-word-entropy WORD)
"
  cset-vec-entropy -- return the entropy for the subset of
  connector-sets associated with WORD, which must be a WordNode.
  This is given by
      sum_d p(w,d) log p(w,d)
  for fixed word w, the sum ranging over all disjuncts.

  The returned entropy is in bits, i.e. computed with log_2.

"
  (pseudo-cset-freq-api 'right-wild-entropy WORD)
)

(define (cset-vec-word-mi WORD)
"
	cset-vec-word-mi - get the fractional mutual information between
   the word and all of it's disjuncts. This is defined as
      MI(w) = (1/p(w)) sum_d p(w,d) log_2 p(w,d)/[p(w,*) p(*,d)]
   This is defined 'fractionally', so that the MI of the dataset
   as a whole can be written as
     MI = sum_w p(w) MI(w)

   Note this function returns MI in units of bits. i.e. log_2
"
	(pseudo-cset-freq-api 'right-wild-fmi WORD)
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------

(define-public (cset-vec-word-lp-dist P WORD-A WORD-B)
"
  cset-vec-word-lp-dist P WORD-A WORD-B - compute the Banache space l_p
  distance between the pseudo-cset vector for WORD-A and WORD-B.

  This is a real metric.  Recall, for p=2, this is just the
  Eucliden distance metric, and for p=1, this is the 'Mahnatten
  distance'.

  WORD-A and WORD-B must both be WordNode's.

  As a metric for measuring the similarity of connector-set vectors,
  this is terrible, because any word that has a lot of observations
  on it will be a long vector, and thus distant from the origin.
  By contrast, any words that have only a few observations will all
  be near the origin, and thus close to one-another, no matter how
  different thier disjuncts are.
"
	(define (subtract TUPLE)  (- (first TUPLE) (second TUPLE)))
	(define subby (add-tuple-math pseudo-cset-api subtract))
	(define normy (add-support-compute subby))
	(normy 'right-lp-norm P (list WORD-A WORD-B))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-cosine WORD-A WORD-B)
"
  cset-vec-cosine WORD-A WORD-B - compute the pseudo-cset vector
  cosine similarity between WORD-A and WORD-B

  WORDs must be WordNodes.
"
	(pseudo-cset-cosine-api 'right-cosine WORD-A WORD-B)
)

; ---------------------------------------------------------------------

(define (get-angle SIM)
	(define pi 3.14159265358979)

	; Stupid-ass guile return a small imaginary number when taking
	; the arccos of 1.0. WTF.  So we need to take the real part!!
	(* 2.0 (/ (real-part (acos SIM)) pi))
)

(define-public (cset-vec-cos-dist WORD-A WORD-B)
"
  cset-vec-cos-dist WORD-A WORD-B - compute the pseudo-cset vector
  cosine distance between WORD-A and WORD-B. The cosine distance
  is defined as dist = 2*arccos(cossim) / pi.  The cosine distance
  obeys the triangle inequality.

  WORDs must be WordNodes.
"
	(get-angle (cset-vec-cosine WORD-A WORD-B))
)

; ---------------------------------------------------------------------

(define-public (cset-vec-jaccard WORD-A WORD-B)
"
  cset-vec-jaccard WORD-A WORD-B - compute the pseudo-cset vector
  Jaccard distance between WORD-A and WORD-B. The Jaccard distance
  is defined as dist = 1 - sim where sim = sum min(a,b)/ sum max(a,b)
  That is,
    sum min(a,b) = sum_d min (N(a,d), N(b,d))
  with the sum ranging over the disjuncts (i.e. on the right).
"
	(pseudo-cset-cosine-api 'right-jaccard WORD-A WORD-B)
)

; ---------------------------------------------------------------------
; Count the number of connectors in the disjunct.
; But the disjunct is just an ConnectorSeq of a bunch of connectors,
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
  is NOT taken!  You probably want to divide by the number of
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

  ITEM can be either a WordNode, or a disjunct (ConnectorSeq).
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

; return a list of all atoms of TYPE
(define (get-all-type TYPE)
	(define all-atoms '())
	(cog-map-type
		(lambda (atom) (set! all-atoms (cons atom all-atoms)) #f)
		TYPE)
	all-atoms
)


(define-public (get-all-csets)
"
  get-all-csets -- Return all of the connector-sets in the atomspace.
"
	(get-all-type 'Section)
)

; ---------------------------------------------------------------------

(define-public (get-all-pseudo-connectors)
"
  get-all-pseudo-connectors -- Return all of the pseudo-connectors
  in the atomspace.
"
	(get-all-type 'Connector)
)

; ---------------------------------------------------------------------
; Example usage:
;
; (use-modules (opencog) (opencog persist) (opencog persist-sql))
; (use-modules (opencog nlp) (opencog nlp learn))
; (sql-open "postgres:///en_pairs_mst?user=linas")
; (sql-open "postgres:///en_pairs_sim?user=linas")
; (fetch-all-words)  <<< 21 secs
; (length (get-all-words))
; 396262
; (define pca (make-pseudo-cset-api))
; (pca 'fetch-pairs)  <<< 295 secs
; (define ac (get-all-cset-words))
; (length ac)
; 49423  (now 37413 in en_pairs_sim)
; (define ad (get-all-disjuncts))
; (length ad)
; 486824 (now 291637 in en_pairs_sim)
;
; (cset-vec-cosine (Word "this") (Word "that"))
; (cset-vec-cosine (Word "he") (Word "she"))
