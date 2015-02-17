; XXX cannot become a module yet unless scm/utilities.scm becomes a module

;(define-module (opencog nlp lg-dict))

;(use-modules (opencog))

; ---------------------------------------------------------------------
; lg-similar? - Handy function to quickly check if two words' LG entries intersect
;
(define-public (lg-similar? word1 word2)
	(define (get-set w)
 		(define roots (filter (lambda (l) (equal? (cog-type l) 'LgWordCset)) (cog-incoming-set w)))
		(map cog-get-partner roots (circular-list w))
	)

	; create the dictionary entry as needed
	(lg-get-dict-entry word1)
	(lg-get-dict-entry word2)
	; check if the two word has common LG dict entry
	(not (null? (lset-intersection equal? (get-set word1) (get-set word2))))
)

; ---------------------------------------------------------------------
; lg-conn-get-type - Get the LgConnectorNode out of LgConnector link
;
(define-public (lg-conn-get-type conn)
	(car (cog-outgoing-set conn))
)

; ---------------------------------------------------------------------
; lg-conn-get-dir - Get the LgConnDirNode out of LgConnector link
;
(define-public (lg-conn-get-dir conn)
	(cadr (cog-outgoing-set conn))
)

