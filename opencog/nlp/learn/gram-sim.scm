;
; gram-sim.scm
;
; Batch-compute the grammatical similarity between all word-pairs.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below compute and store, as a batch process, the
; grammatical similarities between "all possible" pairs of words.
; It assumes that a large number of observations of pseudo-connector
; sets have already been made, and are currently stored in the database.
;
; Note that similarity scores are symmetric, so exchanging left and
; right give the same answer.  Thus, an UnorderedLink is best for
; storing the pair. Specifically, it will be this:
;
;    SimilarityLink
;        WordNode "this"
;        WordNode "that"
;
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog persist) (opencog persist-sql))

(define cos-key (PredicateNode "*-Cosine Distance Key-*"))

; Define the atom at which the cosine similarity value will be stored.
(define (sim-pair WORD-A WORD-B) (SimilarityLink WORD-A WORD-B))

; ---------------------------------------------------------------------
; Public API for fetching the batched results

(define-public (fetch-all-sims)
"
  fetch-all-sims - fetch all SimilarityLinks from the database backend.
"
	(define start-time (current-time))
	(load-atoms-of-type 'SimilarityLink)
	(format #t "Elapsed time to load sims: ~A secs\n"
		(- (current-time) start-time))
)

(define (pair-sim WORD-A WORD-B)
	(cog-link 'SimilarityLink WORD-A WORD-B))

(define-public (sim-cosine SIM)
"
  sim-cosine SIM - return the precomputed cosine similarity for
  the SimilarlityLink for two objects.
"
	(car (cog-value->list (cog-value SIM cos-key)))
)

(define-public (sim-angle-dist SIM)
"
  sim-angle-dist SIM - return the precomputed angular distance
  (arc-cos of the cosine similarity) for the SimilarityLink of
  two items.
"
	(cadr (cog-value->list (cog-value SIM cos-key)))
)

(define-public (word-pair-sim-cosine WORD-A WORD-B)
"
  word-pair-sim-cosine WORD-A WORD-B - return the precomputed cosine
  similarity for WORD-A and WORD-B, if it exists; else return 0.0
"
	(define siml (pair-sim WORD-A WORD-B))
	(if (null? siml) 0.0 (sim-cosine siml))
)

(define-public (word-pair-sim-angle-dist WORD-A WORD-B)
"
  word-pair-sim-angle-dist WORD-A WORD-B - return the precomputed angular
  distance (arc-cos of the cosine similarity) for WORD-A and WORD-B,
  if it exists; else return 1.0
"
	(define siml (pair-sim WORD-A WORD-B))
	(if (null? siml) 1.0 (sim-angle-dist siml))
)

; ---------------------------------------------------------------------
; Compute and store the similarity between the WORD, and the other
; words in the WORD-LIST.  Do NOT actually cache the similarity
; value, if it is less than CUTOFF.  This is used to avoid having
; N-squared pairs cluttering the atomspace.
;
(define (batch-sim WORD WORD-LIST CUTOFF)

	(define cnt 0)

	(define (get-angle SIM)
		(define pi 3.14159265358979)
		; Stupid-ass guile return a small imaginary number when taking
		; the arccos of 1.0. WTF.  So we need to take the real part!!
		(* 2.0 (/ (real-part (acos SIM)) pi))
	)

	(define (set-sim WORD-A WORD-B SIM)
		(cog-set-value!
			(sim-pair WORD-A WORD-B) cos-key
			(FloatValue SIM (get-angle SIM))))

	(for-each
		(lambda (wrd)
			(define sim (cset-vec-cosine WORD wrd))
			(if (and (< CUTOFF sim) (not (equal? WORD wrd)))
				(begin
					(set! cnt (+ 1 cnt))
					(store-atom (set-sim WORD wrd sim)))))
		WORD-LIST)

	cnt
)

; ---------------------------------------------------------------------

; Loop over the entire list of words, and compute similarity scores
; for them.  This might take a very long time!
(define (batch-sim-pairs WORD-LIST CUTOFF)

	(define len (length WORD-LIST))
	(define done 0)
	(define prs 0)
	(define start (current-time))

	; tail-recursive list-walker.
	(define (make-pairs WRD-LST)
		(if (null? WRD-LST) #t
			(begin
				(set! prs (+ prs (batch-sim (car WRD-LST) (cdr WRD-LST) CUTOFF)))
				(set! done (+  done 1))
				(if (eqv? 0 (modulo done 10))
					(let* ((elapsed (- (current-time) start))
							(tot (* done (- len done)))
							(rate (* 0.001 (/ tot elapsed)))
							)
						(format #t
							 "Done ~A/~A Pairs: ~A/~A pct: ~4f Elapsed: ~A rate=~5f K prs/sec\n"
							done len prs tot
							(* 100.0 (/ prs tot))
							elapsed rate
						)))
				(make-pairs (cdr WRD-LST)))))

	(make-pairs WORD-LIST)
)

; If the similarity is less than this, it is not saved.
; The current value is chosen by gut-feel, based on the current
; dataset, which is small/crappy.  The appropriate value is
; likely to be strongly dataset-dependent.
(define cutoff 0.5)

; ---------------------------------------------------------------------
; Example usage:
;
; (use-modules (opencog) (opencog persist) (opencog persist-sql))
; (use-modules (opencog nlp) (opencog nlp learn))
; (sql-open "postgres:///en_pairs_sim?user=linas")
; (sql-open "postgres:///en_pairs_supersim?user=linas")
; (use-modules (opencog cogserver))
; (start-cogserver "opencog2.conf")
; (fetch-all-words)
; (define pca (make-pseudo-cset-api))
; (pca 'fetch-pairs)
; (define ac (get-all-cset-words)))
; (length ac)
; 37413
; (define ad (get-all-disjuncts))
; (length ad)
; 291637
;
; (define firm (filter (lambda (wrd) (< 8.0 (cset-vec-word-len wrd))) ac))
; (length firm)
; 1985
;
; (batch-sim-pairs firm)
