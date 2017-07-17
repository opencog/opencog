;
; en-pairs.scm
;
; Scripts used to generate images for the English word-pair graphs.
;
; als are the stars obj

(define als (add-pair-stars pair-obj))
(define pfrq (add-pair-freq-api als))

; All of the pairs!
(define alp (add-loop-api als))
(define (noop x) x)
(define all-enprs (alp 'map-pair noop)) 
(length all-enprs)   ; 5544578

; Assign each word a score, using SCORE-FN, and then rank them by
; score: i.e. sort them, with highest score first.
(define (score-and-rank SCORE-FN WORD-LIST)
	(sort
		(map (lambda (wrd) (cons (SCORE-FN wrd) wrd)) WORD-LIST)
		(lambda (a b) (> (car a) (car b)))))

(define (score SCORE-FN WORD-LIST)
	(map (lambda (wrd) (cons (SCORE-FN wrd) wrd)) WORD-LIST))

(define (enpr-mi ENPR) (pfrq 'pair-fmi ENPR))

(define scored-enpr-mi (score enpr-mi all-enprs))

(define binned-enpr-mi (bin-count-simple scored-enpr-mi 400))

(let ((outport (open-file "/tmp/binned-enpr-mi.dat" "w")))
	(print-bincounts-tsv binned-enpr-mi outport)
	(close outport))



; binned-cset-mi and weighted-cset-mi


