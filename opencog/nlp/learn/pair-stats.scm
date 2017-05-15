;
; pair-states.scm
;
; Return assorted database statistics, pertaining to word-pairs.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; The statistics reported here are those collected via the code
; in `link-pipeline.scm` and computed in `compute-mi.scm`.  Therefore
; structure defintions there and here need to be maintained
; consistently.
;
; Many or most of the stats returned here assume that the pair-counting
; batch job has completed. They get stats based on the current contents
; of the atomspace.
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (opencog))
(use-modules (opencog persist))

; ---------------------------------------------------------------------
(define-public (get-sentence-count)
"
  get-sentence-count -- get the number of sentences observed.
  This does fetch the count from the database.

  This count is maintained by the link-pipeline code.
"
	(get-count (fetch-atom (SentenceNode "ANY")))
)

(define-public (get-parse-count)
"
  get-parse-count -- get the number of parses observed.
  This does fetch the count from the database.

  This count is maintained by the link-pipeline code.
"
	(get-count (fetch-atom (ParseNode "ANY")))
)

(define-public (avg-sentence-length)
"
  avg-sentence-length -- get expected value for the number of words
  in the sentence.
"
	; Each word is counted once, in every parse.
	(/ (total-word-observations) (get-parse-count))
)

; ---------------------------------------------------------------------

; Call the function FUNC on the word-pair, but only if the
; word-pair exists. This prevents garbaging up the atomspace
; with bogus word-pair lists.
(define (safety-wrap FUNC WORD-A WORD-B)
	(define pair
		(if
			(and
				(not (null? WORD-A))
				(not (null? WORD-B)))
			(cog-link 'ListLink WORD-A WORD-B)
			'()))
	(if (not (null? pair))
		(FUNC pair)
		#f)
)

(define (count-clique-pair PAIR)
"
  Return count for the clique-pair PAIR
  PAIR should be a ListLink of two words.
"
	(define evl (cog-link 'EvaluationLink pair-pred PAIR))
	(if (null? evl) 0 (get-count evl))
)

(define (count-dist-pair PAIR)
"
  Return sum over all counts of the distance pairs.
  This should, in all cases, return the same value as
  `count-clique-pair`, above, since the sum over distances
  should equal the total number of observations. This can be
  checked with the `verify-clique-pair-sums` function below.

  PAIR should be a ListLink of two words.
"
	(fold
		(lambda (ex sum) (+ (get-count ex) sum))
		0
		(filter
			(lambda (lnk) (equal? pair-dist (gar lnk)))
				(cog-incoming-by-type PAIR 'ExecutionLink)))
)

(define (avg-dist-pair PAIR)
"
  Return average distance over all counts of the distance pairs.
  PAIR should be a ListLink of two words.
"
	(define (get-dist exl)
		(string->number (cog-name (caddr (cog-outgoing-set exl))))
	)
	(/
		(fold
			(lambda (ex sum) (+ (* (get-count ex) (get-dist ex)) sum))
			0
			(filter
				(lambda (lnk) (equal? pair-dist (gar lnk)))
					(cog-incoming-by-type PAIR 'ExecutionLink)))

		(count-dist-pair PAIR))
)

(define-public (avg-dist-word-pair WORD-A WORD-B)
"
  Return average distance over all counts of the distance pairs
"
	(define dist (safety-wrap avg-dist-pair WORD-A WORD-B))
	(if dist dist 1e40)
)

; ---------------------------------------------------------------------

(define-public (verify-clique-pair-sums PAIR-LIST)
"
  This checks consistency of the the clique-pair total count, with
  the subcounts of each pair, accodring to the distance between
  the words. The sum of the subtotals should equal the total.
  It should not throw.

  Example usage: (verify-clique-pair-sums (get-all-clique-pairs))
"
	(define cnt 0)
	(for-each
		(lambda (PAIR)
			(set! cnt (+ cnt 1))
			(if (not (eqv? (count-dist-pair PAIR) (count-clique-pair PAIR)))
				(throw 'bad-count 'foobar PAIR)
				(format #t "Its OK ~A\n" cnt)
			))
		PAIR-LIST)
)

; ---------------------------------------------------------------------
; misc utilities of research interest

(define-public (get-left-word-of-pair PAIR)
"
  get-left-word-of-pair PAIR -- Given an EvaluationLink PAIR
  holding a word-pair, return the word on the left.
"
	(gadr PAIR)
)

(define-public (get-right-word-of-pair PAIR)
"
  get-right-word-of-pair PAIR -- Given the EvaluationLink PAIR
  holding a word-pair, return the word on the right.
"
	(gddr PAIR)
)

(define-public (get-all-pairs)
"
  get-all-pairs - return a list holding all of the observed word-pairs
  Caution: this can be tens of millions long!
"
	; The list of pairs is mostly just the incoming set of the ANY node.
	; However, this does include some junk, sooo ... hey, both left and
	; right better be words.
	(filter!
		(lambda (pair)
			(and
				(equal? 'WordNode (cog-type (get-left-word-of-pair pair)))
				(equal? 'WordNode (cog-type (get-right-word-of-pair pair)))))
		(cog-incoming-by-type any-pair-pred 'EvaluationLink))
)

(define-public (get-all-clique-pairs)
"
  get-all-clique-pairs - return a list holding all of the observed
  word-pairs. Caution: this can be tens of millions long, and take
  dozens of minutes to compute!
"
	; The list of pairs is mostly just the incoming set of the ANY node.
	; However, this does include some junk, sooo ... hey, both left and
	; right better be words.
	(filter!
		(lambda (pair)
			(and
				(equal? 'WordNode (cog-type (get-left-word-of-pair pair)))
				(equal? 'WordNode (cog-type (get-right-word-of-pair pair)))))
		(cog-incoming-by-type pair-pred 'EvaluationLink))
)

(define-public (total-word-observations)
"
  total-word-observations -- return a total of the number of times
  any/all words were observed.  That is, compute and return N(*),
  as defined above, and in the diary.  This does NOT work from a
  cached value.  Also, this does NOT fetch atoms from the database!
"
   (get-total-atom-count (get-all-words))
)

(define-public (total-pair-observations GET-PAIR)
"
  total-pair-observations GET-PAIR -- return a total of the number
  of times that word-pairs were observed. That is, return N(*,*),
  as defined above, and in the diary.

  The argument GET-PAIR indicates what kind of pairs to count,
  as there are several kinds that are being tracked.  Currently,
  the only valid values for GET-PAIR are `get-any-pair` and
  `get-clique-pair`.
"
	; Just get the previously computed amount.
	(get-count (GET-PAIR (ListLink any-left any-right)))
)

(define-public (get-pair-frequency GET-PAIR LEFT-ITEM RIGHT-ITEM)
"
  get-pair-frequency GET-PAIR LEFT-ITEM RIGHT-ITEM -- return the
  frequency of observing the pair (LEFT-ITEM, RIGHT-ITEM).
  Return zero if the pair has not been observed.

  Currrently, the two ITEMs should be WordNodes, and GET-PAIR
  should be a function that returns the atom where the precomputed
  counts are stored.

  Currently, the only valid values for GET-PAIR are `get-any-pair`
  and `get-clique-pair`.
"
	(define cnt
		(safety-wrap
			(lambda (pr) (get-count (GET-PAIR pr)))
			 LEFT-ITEM RIGHT-ITEM))
	(if cnt (/ cnt (total-pair-observations GET-PAIR)) 0)
)

(define-public (get-left-observations GET-PAIR ITEM)
"
  get-left-observations GET-PAIR ITEM
  Return the number of times that ITEM occurs in the left side
  of the GET-PAIR relationship. That is, return the summed
  count N(w, *), as defined above, and in the diary.

  Currently, ITEM is assumed to be a WordNode.

  Currently, the only valid values for GET-PAIR are `get-any-pair`
  and `get-clique-pair`.
"
	(define pair (cog-link 'ListLink ITEM any-right))
	;; the wildcard is on the right.
	(get-count (GET-PAIR (ListLink ITEM any-right)))
)

(define-public (get-word-count-str WORD-STR)
"
  get-word-count-str WORD-STR
  Return the number of times that WORD-STR has ben observed. That is,
  return N(w) as defined in the diary. Here, w is WORD-STR, assumed to
  be a string.
"
	(get-count (WordNode WORD-STR))
)

(define-public (get-total-cond-prob ALL-PAIRS)
"
  get-total-cond-prob ALL-PAIRS -- return the total conditional
  probability of seeing the all word-pairs.  That is, return the
  sum over left and right words w_l, w_r of  N(w_l, w_r) / (N(w_l) N(w_r))
XXXX this is wrong because its incorrectly normalized.

  Contrast this result with that of get-total-pair-prob
"
	; Return N(w_l, w_r) / N(w_l) N(w_r)
	(define (term pair)
		(/ (get-count pair)
			(* (get-count (get-left-word-of-pair pair))
				(get-count (get-right-word-of-pair pair)))))

	; textbook tail-recursive solution.
	(define (term-sum lst cnt)
		(if (null? lst) cnt
			(term-sum (cdr lst) (+ cnt (term (car lst))))))

	(term-sum ALL-PAIRS 0)
)

(define-public (get-total-pair-prob ALL-PAIRS)
"
  get-total-pair-prob - return the total pair-wise conditional
  probability of seeing a word-pair.  That is, return the sum over
  left and right words w_l, w_r of
      N(w_l, w_r) / (N(w_l, *) N(*, w_r))

XXXX this is wrong because its incorrectly normalized.
  Contrast this result with that of get-total-cond-prob
"

	; Return N(w_l, w_r) / N(w_l) N(w_r)
	(define (term pair)
		(/ (get-count pair)
			(* (get-count (get-left-word-of-pair pair))
				(get-count (get-right-word-of-pair pair)))))

	; textbook tail-recursive solution.
	(define (term-sum lst cnt)
		(if (null? lst) cnt
			(term-sum (cdr lst) (+ cnt (term (car lst))))))

	(term-sum ALL-PAIRS 0)
)

; ---------------------------------------------------------------------
