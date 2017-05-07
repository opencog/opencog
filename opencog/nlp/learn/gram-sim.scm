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

; If the similarity is less than this, it is not saved.
; The current value is chosen by gut-feel, based on the current
; dataset, which is small/crappy.  The appropriate value is
; likely to be strongly dataset-dependent.
(define cutoff 0.5)

; Define the atom at which the cosine similarity value will be stored.
(define (sim-pair WORD-A WORD-B) (SimilarityLink WORD-A WORD-B))

; Compute and store the similarity between the WORD, and the other
; words in the word-list
(define (batch-sim WORD WORD-LIST)

	(define num-checked 0)
	(define num-stored 0)

	(define (store-sim WORD-A WORD-B SIM)
		(set! num-stored (+ num-stored 1))
		(format #t "~A ~A Similarity ~A <--> ~A = ~A\n"
			num-checked num-stored (cog-name WORD-A) (cog-name WORD-B) SIM)
		(store-atom
			(cog-set-value!
				(sim-pair WORD-A WORD-B) cos-key
				(FloatValue SIM (get-angle SIM)))))

	(for-each
		(lambda (wrd)
			(define sim (cset-vec-cosine WORD wrd))
			(set! num-checked (+ num-checked 1))
			(if (and (< cutoff sim) (not (equal? WORD wrd)))
				(store-sim WORD wrd sim)))
		WORD-LIST)

	; print some progress info.
	(format #t "Word ~A had ~A sims on ~A (~A pct)\n"
		(cog-name WORD) num-stored (length WORD-LIST)
		(/ (* 100.0 num-stored) (length WORD-LIST)))
)

; ---------------------------------------------------------------------

; Loop over the entire list of words, and compute similarity scores
; for them.  This might take a very long time!
(define (batch-sim-pairs WORD-LIST)

	(define len (length WORD-LIST))
	(define done 0)

	; tail-recursive list-walker.
	(define (make-pairs WRD-LST)
		(if (null? WRD-LST) #t
			(begin
				(set! done (+  done 1))
				(format #t "Doing ~A of ~A\n" done len)
				(batch-sim (car WRD-LST) (cdr WRD-LST))
				(make-pairs (cdr WRD-LST)))))

	(make-pairs WORD-LIST)
)

; ---------------------------------------------------------------------
; Example usage:
;
; (use-modules (opencog) (opencog persist) (opencog persist-sql))
; (use-modules (opencog nlp) (opencog nlp learn))
; (sql-open "postgres:///en_pairs_sim?user=linas")
; (use-modules (opencog cogserver))
; (start-cogserver "opencog2.conf")
; (fetch-all-words)
; (fetch-pseudo-csets (get-all-words))
; (define ac (filter-words-with-csets (get-all-words)))
; (length ac)
; 37413
; (define ad (get-all-disjuncts))
; (length ad)
; 291637
;
; (define firm (filter (lambda (wrd) (< 8.0 (cset-vec-len wrd))) ac))
; (length firm)
; 1985
;
; (batch-sim-pairs firm)
