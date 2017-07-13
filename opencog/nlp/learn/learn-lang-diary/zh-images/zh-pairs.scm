;
; zh-pairs.scm
;
; Scripts used to generate images for the mandarin chinese graphs.
;
; als are the stars obj

(define als (add-pair-stars pair-obj))
(define pfrq (add-pair-freq-api als))

(define ala (make-any-link-api))

; All of the pairs!
(define all-hanprs (ala 'all-pairs))

(define (hanpr-mi HANPR) (pfrq 'pair-fmi HANPR))

(define sorted-hanpr-mi (score-and-rank hanpr-mi all-hanprs))


; binned-cset-mi and weighted-cset-mi


