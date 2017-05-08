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

; Return string holding tab-separated table of rankings
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
; A sorted list of score-word pairs, where the score is the cset length
(define sorted-lengths (score-and-rank cset-vec-len all-cset-words)

(let ((outport (open-file "/tmp/ranked-lengths.dat" "w")))
	(print-ts-rank sorted-lengths outport)
	(close outport))

; Consider, for example, the length-squared, divided by the number
; of observations.
(define (lensq-vs-obs wrd)
	(define len (cset-vec-len wrd))
	(/ (* len len) (cset-vec-observations wrd)))

; The legnth vs observation ranking; but discard everything
; with a small number of observations.
(define sorted-lensq-norm
	(score-and-rank lensq-vs-obs
	 	 (filter (lambda (wrd) (< 100 (cset-vec-observations wrd)))
			all-cset-words)))

(let ((outport (open-file "/tmp/ranked-sqlen-norm.dat" "w")))
	(print-ts-rank sorted-lensq-norm outport)
	(close outport))

