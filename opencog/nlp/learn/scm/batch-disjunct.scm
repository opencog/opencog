;
; batch-disjunct.scm
;
; Batch-compute marginals needed for the symmetric word-MI.
;
; Copyright (c) 2018 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; Given a word, and a vector attached to that word, one can compute a
; symmetric mutual-information (MI) value betweem two words. In the
; typical example, the vector consists of counts observed on disjuncts
; associated to that word (although it could be any vector).
;
; This file precomputes the various partial sums (marginals) that occur
; in the definition of the symmetric MI.  It assumes that the vectors
; (i.e. observation counts) are in an SQL database; it fetches those,
; computes the assorted marginals, and writes those marginals back to
; disk. Specifically, the marginals are those needed for the
; `transpose-api`, which is used to compute the symmetric MI.
;
; REVIEW
; ------
; The symmetric-MI is described in the diary. What follows is a quick
; review that motivates the stuff in this file.
;
; One has numbers (observation counts) `N(w,d)` which count the number
; of times that disjuct `d` was observed on word `w`.  These numbers
; define a vector on the word `w`. Given two words `w` and `u`, one
; may define the vector dot-product between them as:
;
;      S(u,w) = sum_d N(u,d) N(w,d)
;
; Note that this product is symmetric: S(u,w) = S(w,u)
; Note that this product underlies the definition of the cosine between
; the two vectors.  The cosine is just
;
;     cos(u,w) = S(u,w) / sqrt (S(u,u) S(w,w))
;
; This product can also be used to define a symmetric mutual
; information between the words:
;
;     MI(u,w) = log_2 S(u,w) S(*,*) / S(u,*) S(w,*)
;
; where, as usual, the star * defines a wild-card sum:
;
;     S(u,*) = S(*,u) = sum_w S(u,w)
;
; The above follows because one can interpret
;
;     P(u,w) = S(u,w) / S(*,*)
;
; as a probability of observing the pair (u,w).  All the usual notions
; of probability then ensue, and MI(u,w) is just the usual definition
; of the (symmetric) mutual information.
;
; Note that S(u,w) can itself be thought of as a matrix; it is a product
; of N times its transpose.  That is, let [N] denote the matrix whose
; matrix elements are N(w,d), and [S] be thw matrix whose matrix
; elements are S(u,w). Then one has that
;
;    [S] = [N][N]^T
;
; where [N]^T is the matrix-transpose, i.e. is the matrix whose matrix
; elements are N^T(d,w)=N(w,d).
;
; COMPUTATIONS
; ------------
; The definition of MI(u,w) above requires the partial sums for S(u,*)
; and S(*,*). These can be fetched from the atomspace by the
; `add-transpose-api` matrix class. But, to be fetched, they need to
; first be precomputed by the `add-transpose-compute` matrix class.
; This class, in turn relies on the `add-support-api` class, which
; requires that the support marginals were previously computed by the
; `add-support-compute` class.  Thus, this does all the bulk
; calculations that generate these marginals, and then stores them back
; to disk.
;
; Note that the scripts here are specifically tailored to doing just
; one "side" of the computation: viz, only [N][N]^T or only [N]^T[N].
; That is because typically, one is interested only in the one, or
; in the other, but not both. Computing both takes a lot more CPU time.
; Even worse, the number of disjuncts `d` far exceeds the number of
; words `w`, by more than an order of magnitude, and sometimes two,
; and so the sizes of these matrixes are vastly different, and the
; resulting computations waste not only CPU but also RAM and storage.
; Thus, only one "side" is computed here.
;
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (opencog))
(use-modules (opencog persist))
(use-modules (opencog matrix))

; ---------------------------------------------------------------------

(define-public (batch-transpose LLOBJ)
"
  batch-transpose - bulk-compute transpose marginals.

  This computes the marginals needed to get the `add-transpose-api`
  to work correctly.  This is quite CPU-intensive. The resulting
  marginals are saved to the database, after they are computed.
"
	(let* ((star-obj     (add-pair-stars LLOBJ))
			(support-obj   (add-support-api star-obj))
			(scomp-obj     (add-support-compute star-obj))
			(centr-obj     (make-central-compute star-obj))
			(store-obj     (make-store star-obj))
			(trans-obj     (add-transpose-compute star-obj))
		)

		; -------------
		; If the marginal counts have not yet been computed, do so now.
		(define (batch-left-support)
			; The cross-objects need to have the stars created, before
			; they can be used.
			(if (LLOBJ 'provides 'make-left-stars)
				(LLOBJ 'make-left-stars))

			; 'mmt-marginals loops over 'left-basis and for each
			; of those, loops over 'right-duals and calls 'left-count
			; on the support-obj for each of the duals. This was saved
			; on the 'left-wildcard on the dual.  That means that we
			; need to have the left-marginals all computed.
			(scomp-obj 'left-marginals)
			(centr-obj 'cache-left)
			(store-obj 'store-left-marginals)
		)

		; This assumes that pairs have been fetched already.
		(define (batch-mmt-marginals)

			; If left supports have not yet been computed, then
			; do so now. We can tell if they have been, by simply
			; accessing a quantity we expect to have already.
			(catch #t (lambda () (support-obj 'total-support-left))
				(lambda (key . args)
					(batch-left-support)))

			; 'mmt-marginals loops over 'left-basis and records
			; them on 'right-wildcard.  Thus, we need to save
			; the 'right-wildcard to disk.
			(trans-obj 'mmt-marginals)
			(store-obj 'store-right-marginals)
			(display "Done computing and saving sum_y N(x,y) N(*,y)\n")
		)

		; -------------
		; If the marginal counts have not yet been computed, do so now.
		(define (batch-right-support)
			; The cross-objects need to have the stars created, before
			; they can be used.
			(if (LLOBJ 'provides 'make-right-stars)
				(LLOBJ 'make-right-stars))

			(scomp-obj 'right-marginals)
			(centr-obj 'cache-right)
			(store-obj 'store-right-marginals)
		)

		; This assumes that pairs have been fetched already.
		(define (batch-mtm-marginals)

			; If right supports have not yet been computed, then
			; do so now. We can tell if they have been, by simply
			; accessing a quantity we expect to have already.
			(catch #t (lambda () (support-obj 'total-support-right))
				(lambda (key . args)
					(batch-right-support)))

			(trans-obj 'mtm-marginals)
			(store-obj 'store-left-marginals)
			(display "Done computing and saving sum_x N(x,y) N(x,*)\n")
		)

		; -------------
		; Methods on this class.
		(lambda (message . args)
			(case message
				((mmt-marginals)  (batch-mmt-marginals))
				((mtm-marginals)  (batch-mtm-marginals))
				(else             (apply LLOBJ (cons message args))))
			)))

; ---------------------------------------------------------------------
; Example usage
;
; (define cva (make-connector-vec-api))
; (define cvs (add-pair-stars cva))
; (define cvp (add-support-api cvs))
; (define btc (batch-transpose cva))
; (btc 'batch-cross)
