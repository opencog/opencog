;
; disjunct-stats.scm
;
; Assorted ad-hoc collection of tools for understanding the
; word-simillarity obtained via pseudo-disjunct overlap.
; These were used to create the section "Connetor Sets 7 May 2017"
; in the diary.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; Ranking and printing utilities
;
; Assign each word a score, using SCORE-FN, and then rank them by
; score: i.e. sort them, with highest score first.
(define (score-and-rank SCORE-FN WORD-LIST)
	(sort
		(map (lambda (wrd) (cons (SCORE-FN wrd) wrd)) WORD-LIST)
		(lambda (a b) (> (car a) (car b)))))

; Print to port a tab-separated table of rankings
(define (print-ts-rank scrs port)
	(define cnt 0)
	(for-each
		(lambda (pr)
			(set! cnt (+ cnt 1))
			(format port "~A	~A	\"~A\"\n" cnt (car pr) (cog-name (cdr pr))))
		scrs))

; ---------------------------------------------------------------------
; A list of all words that have csets.
(define all-cset-words (filter-words-with-csets (get-all-words)))

; ---------------------------------------------------------------------
; A sorted list of score-word pairs, where the score is the cset
; observations. Note that the score is *identical* to the number of
; times that the word was observed during MSt parsing. That is because
; exactly one disjunct is extracted per word, per MST parse.
(define sorted-obs (score-and-rank cset-vec-observations all-cset-words)

; Print above to a file, so that it can be graphed.
(let ((outport (open-file "/tmp/ranked-observations.dat" "w")))
	(print-ts-rank sorted-obs outport)
	(close outport))

; ---------------------------------------------------------------------
; Compute the average number of observations per disjunct.
(define (avg-obs WORD)
	(/ (cset-vec-observations WORD) (cset-vec-support WORD)))

; Compute the average number of observations per disjunct.
; Discard anything with less than 100 observations.
(define sorted-avg
	(score-and-rank avg-obs
		(filter (lambda (wrd) (< 100 (cset-vec-observations wrd)))
			all-cset-words)))

(let ((outport (open-file "/tmp/ranked-avg.dat" "w")))
	(print-ts-rank sorted-avg outport)
	(close outport))

; ---------------------------------------------------------------------
; A sorted list of the support of a word.
; The support is simply how many basis elements of a vector are
; non-zero.  Equivalently, it is the size of the set of unique
; disjuncts associated with a word (counted without multiplicity).

(define sorted-support (score-and-rank cset-vec-support all-cset-words))

(let ((outport (open-file "/tmp/ranked-support.dat" "w")))
	(print-ts-rank sorted-support outport)
	(close outport))

; ---------------------------------------------------------------------
; A sorted list of score-word pairs, where the score is the cset length

(define sorted-lengths (score-and-rank cset-vec-len all-cset-words)

(let ((outport (open-file "/tmp/ranked-lengths.dat" "w")))
	(print-ts-rank sorted-lengths outport)
	(close outport))

; ---------------------------------------------------------------------
; Consider, for example, the length-squared, divided by the number
; of observations.
(define (lensq-vs-obs wrd)
	(define len (cset-vec-len wrd))
	(/ (* len len) (cset-vec-observations wrd)))

; The length vs observation ranking; but discard everything
; with a small number of observations.
(define sorted-lensq-norm
	(score-and-rank lensq-vs-obs
		(filter (lambda (wrd) (< 100 (cset-vec-observations wrd)))
			all-cset-words)))

(let ((outport (open-file "/tmp/ranked-sqlen-norm.dat" "w")))
	(print-ts-rank sorted-lensq-norm outport)
	(close outport))

; ---------------------------------------------------------------------
; RMS deviation from mean.
;  sum_i (x-a)^2 = sum_i x^2 - 2a sum_i x + a^2 sum_i
; but now divide by N = sum_i, again, to get
;  sum_i (x-a)^2 = <x^2> - a^2
;
(define (avg-obs WORD)
	(/ (cset-vec-observations WORD) (cset-vec-support WORD)))

(define (meansq-obs WORD)
	(define len (cset-vec-len WORD))
	(/ (* len len) (cset-vec-support WORD)))

(define (rms-deviation WORD)
	(define avg (avg-obs WORD))
	(sqrt (- (meansq-obs WORD) (* avg avg))))

; Compute the RMS deviation from the average number of observations
; per disjunct.
; Discard anything with less than 100 observations.
(define sorted-rms
	(score-and-rank rms-deviation
		(filter (lambda (wrd) (< 100 (cset-vec-observations wrd)))
			all-cset-words)))

(let ((outport (open-file "/tmp/ranked-rms.dat" "w")))
	(print-ts-rank sorted-rms outport)
	(close outport))

;
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Hubbbiness.

(define (avg-con-count wrd)
	(/ (cset-vec-connectors wrd) (cset-vec-observations wrd)))

; Rank by average disjunct size; but discard everything
; with a small number of observations.
(define sorted-avg-connectors
	(score-and-rank avg-con-count
		(filter (lambda (wrd) (< 100 (cset-vec-observations wrd)))
			all-cset-words)))

(define sorted-avg-connectors
	(score-and-rank avg-con-count all-cset-words))

(define sorted-avg-connectors
	(score-and-rank avg-con-count
		(filter (lambda (wrd) (< 300 (cset-vec-observations wrd)))
			all-cset-words)))

(let ((outport (open-file "/tmp/ranked-avg-connectors.dat" "w")))
	(print-ts-rank sorted-avg-connectors outport)
	(close outport))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
;
; Distributions for two particular words.

(define dj-united (score-and-rank get-count
		(get-cset-vec (Word "United"))))

(define dj-it (score-and-rank get-count
		(get-cset-vec (Word "It"))))

; Print to port a tab-separated table of rankings
(define (print-ts-dj-rank cset-a cset-b port)
	(define idx 0)
	(for-each
		(lambda (pra prb)
			(set! idx (+ idx 1))
			(format port "~A	~A	~A\n" idx (car pra) (car prb)))
		cset-a cset-b))

(let ((outport (open-file "/tmp/ranked-dj-united.dat" "w")))
	(print-ts-dj-rank dj-united dj-it outport)
	(close outport))

; ---------------------------------------------------------------------
; Sum over distributions

; Create and zero out array.
(define dj-sum (make-array 0 312500))

; Create a ranked list of disjuncts for WORD
(define (make-ranked-dj-list WORD)
	(score-and-rank get-count (get-cset-vec WORD)))

; Accumulate the disjunct counts for WORD, into dj-sum
(define (accum-dj-counts WORD)
	(define idx 0)
	(for-each
		(lambda (pr)
			(array-set! dj-sum (+ (car pr) (array-ref dj-sum idx)) idx)
			(set! idx (+ idx 1)))
		(make-ranked-dj-list WORD)))

; Accumulate the disjunct counts for all the words in the word-list
(define (accum-dj-all WORD-LIST)
	(for-each
		(lambda (word) (accum-dj-counts word))
		WORD-LIST))

; Accumulate all, but only for those with 100 or more observations.
(accum-dj-all
	(filter
		(lambda (word) (< 100 (cset-vec-observations word)))
		all-cset-words))

(define (print-dj-acc port)
	(define idx 0)
	(for-each
		(lambda (cnt)
			(set! idx (+ idx 1))
			(if (< 0 cnt)
				(format port "~A	~A\n" idx cnt)))
		(array->list dj-sum)))

(let ((outport (open-file "/tmp/ranked-dj-counts.dat" "w")))
	(print-dj-acc outport)
	(close outport))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Entropies.

; Several ways of counting the same thing. Thse should all
; give the same result.
(define total-cset-count (cset-observations all-cset-words))
(define total-cset-count (get-total-cset-count))

; Get the "partial entropy".
(define (partial-entropy LIST)
	(fold
		(lambda (item sum)
			(define pitem (/ (cset-vec-observations item) total-cset-count))
			(- sum (* pitem (log pitem))))
		0
		LIST))

; Get the "word entropy".
(define word-entropy (partial-entropy all-cset-words))
(define word-entropy-bits (/ word-entropy (log 2.0)))

(define all-disjuncts (get-all-disjuncts))
(define disjunct-entropy (partial-entropy all-disjuncts))
(define disjunct-entropy-bits (/ disjunct-entropy (log 2.0)))

(define cset-entropy
	(fold
		(lambda (word sum) (+ sum  (cset-vec-entropy word)))
		0
		all-cset-words))

(define cset-entropy-bits (/ cset-entropy (log 2.0)))

; ---------------------------------------------------------------------
