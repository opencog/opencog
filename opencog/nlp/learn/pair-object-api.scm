
; The GET-PAIR argument should be a function that returns all atoms
; that hold counts for a pair (ListLink left-item right-item). It
; should return a list (possibly the empty list).  Note that in some
; cases, the list may be longer than just one item, e.g. for schemas
; that count link lengths.  In thise case, the total count for the
; pair is taken to be the sum of the individual counts on all the
; atoms in the list.


; Additional documentation for what this is TBD

(define (make-pair-object-api-example) 
	(let () 
		; Return the atom holding the count.
		(define (get-pair PAIR) "foobar")

		; Return a list of atoms hold the count.
		(define (get-pairs PAIR) "foobar")

		(define (get-left-wildcard WORD) "foobar")

		(define (get-right-wildcard WORD) "foobar")

		(define (get-wild-wild) "foobar")

	; Methods on the object
	; Example: (OBJ 'get-left-wildcard WORD) calls the
	; get-left-wildcard method, passing WORD as the object.
	(lambda (message . args) 
		(apply (case message 
				((get-pair) get-pair) 
				((get-left-wildcard) get-left-wildcard) 
				((get-right-wildcard) get-right-wildcard) 
				((get-wild-wild) get-wild-wild) 
				(else (error "Bad method call on ANY-link"))) 
			args)))) 
