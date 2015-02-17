;(define-module (opencog nlp lg-dict))

;(use-modules (opencog))
;(use-modules (opencog extension))

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
(define (lg-conn-get-type conn)
	(gar conn)
)

; ---------------------------------------------------------------------
; lg-conn-get-dir - Get the LgConnDirNode out of LgConnector link
;
(define (lg-conn-get-dir conn)
	(gdr conn)
)

; ---------------------------------------------------------------------
; lg-conn-type-match? - Check if two connectors type match
;
(define (lg-conn-type-match? lconn rconn)
	; helper function to do special handling of head/tail
	(define (match-start lconn-list rconn-list)
		(define lh (car lconn-list))
		(define rh (car rconn-list))
		(cond
			((and (char-lower-case? lh) (char-lower-case? rh))
				(if (equal? lh rh)
					#f
					(match-subscript (cdr lconn-list) (cdr rconn-list))
				)
			)
			((char-lower-case? lh)
				(match-subscript (cdr lconn-list) rconn-list)
			)
			((char-lower-case? rh)
				(match-subscript lconn-list (cdr rconn-list))
			)
			(else
				(match-subscript lconn-list rconn-list)
			)
		)
	)
	; main function for matching subscripts
	(define (match-subscript lconn-list rconn-list)
		(cond
			((or (null? lconn-list) (null? rconn-list))
				#t
			)
			((or (char-upper-case? (car lconn-list)) (char-upper-case? (car rconn-list)))
				(if (equal? (car lconn-list) (car rconn-list))
					(match-subscript (cdr lconn-list) (cdr rconn-list))
					#f
				)
			)
			((equal? (car lconn-list) (car rconn-list))
				(match-subscript (cdr lconn-list) (cdr rconn-list))
			)
			((or (equal? (car lconn-list) #\*) (equal? (car rconn-list) #\*))
				(match-subscript (cdr lconn-list) (cdr rconn-list))
			)
			(else
				#f
			)
		)
	)
	(match-start
		(string->list (cog-name (lg-conn-get-type lconn)))
		(string->list (cog-name (lg-conn-get-type rconn)))
	)
)

; ---------------------------------------------------------------------
; lg-conn-linkable? - Check if two connectors can be linked
;
; The two connectors must have different directions, but does not matter
; which one is + and which one is -.
;
(define (lg-conn-linkable? conn1 conn2)
	; check if the two connectors type match
	(if (lg-conn-type-match? conn1 conn2)
		; check if the directions are correct
		(not (equal? (lg-conn-get-dir conn1) (lg-conn-get-dir conn2)))
		#f
	)
)

