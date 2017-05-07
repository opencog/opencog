;
; disjunct-stats.scm
;
; Assorted ad-hoc collection of tools for understanding the
; word-simillarity obtained via pseudo-disjunct overlap.
;

; A list of all words that have csets.
(define all-cset-words (filter-words-with-csets (get-all-words)))

; A sorted list of score-word pairs, where the score is the cset length
(define sorted-lengths
	(sort
		(map (lambda (wrd) (cons (cset-vec-len wrd) wrd)) all-cset-words)
		(lambda (a b) (> (car a) (car b)))))

; A sorted list of score-word pairs, where the score is the cset
; observations
(define sorted-obs
	(sort
		(map (lambda (wrd) (cons (cset-vec-observations wrd) wrd)) all-cset-words)
		(lambda (a b) (> (car a) (car b)))))

; Return string holding tab-separated table of rankings
(define (print-ts-rank scrs port)
	(define cnt 0)
	(for-each
		(lambda (pr)
			(set! cnt (+ cnt 1))
			(format port "~A	~A	\"~A\"\n" cnt (car pr) (cog-name (cdr pr))))
		scrs))

(let ((outport (open-file "/tmp/ranked-observations.dat" "w")))
	(print-ts-rank sorted-obs outport)
	(close outport))

(let ((outport (open-file "/tmp/ranked-lengths.dat" "w")))
	(print-ts-rank sorted-lengths outport)
	(close outport))

