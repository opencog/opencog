;
; zh-pairs.scm
;
; Scripts used to generate images for the mandarin chinese graphs.
;
; als are the stars obj

(define als (add-pair-stars pair-obj))
(define pfrq (add-pair-freq-api als))

; All of the pairs!
(define ala (make-any-link-api))
(define all-hanprs (ala 'all-pairs))  ; 5922477

; Alternate algo
(define alp (add-loop-api als))
(define (noop x) x)
(define all-hanprs (alp 'map-pair noop))

; Assign each word a score, using SCORE-FN, and then rank them by
; score: i.e. sort them, with highest score first.
(define (score-and-rank SCORE-FN WORD-LIST)
	(sort
		(map (lambda (wrd) (cons (SCORE-FN wrd) wrd)) WORD-LIST)
		(lambda (a b) (> (car a) (car b)))))

(define (score SCORE-FN WORD-LIST)
	(map (lambda (wrd) (cons (SCORE-FN wrd) wrd)) WORD-LIST))

(define (hanpr-mi HANPR) (pfrq 'pair-fmi HANPR))

(define scored-hanpr-mi (score hanpr-mi all-hanprs))

(define binned-hanpr-mi (bin-count-simple scored-hanpr-mi 400))

(let ((outport (open-file "/tmp/binned-hanpr-mi.dat" "w")))
	(print-bincounts-tsv binned-hanpr-mi outport)
	(close outport))



; binned-cset-mi and weighted-cset-mi


