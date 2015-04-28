scm
;===========================
; wire-order tests
; test that the results of wiring are independent of the order 
; of the operations.

(define (wire-it-a)
	(define sentences (make-wire "up-wire"))
	(define sentence-parts (make-wire "down-wire"))

	(cgw-incoming sentence-parts sentences)
	(wire-probe "sent-list" sentence-parts)
	(cgw-source-atoms sentences 'SentenceNode)
)

(define (wire-it-b)
	(define sentences (make-wire "up-wire"))
	(define sentence-parts (make-wire "down-wire"))

	(cgw-incoming sentence-parts sentences)
	(cgw-source-atoms sentences 'SentenceNode)
	(wire-probe "sent-list" sentence-parts)
)

(define (wire-it-c)
	(define sentences (make-wire "up-wire"))
	(define sentence-parts (make-wire "down-wire"))

	(wire-probe "sent-list" sentence-parts)
	(cgw-source-atoms sentences 'SentenceNode)
	(cgw-incoming sentence-parts sentences)
)

(define (wire-it-d)
	(define sentences (make-wire "up-wire"))
	(define sentence-parts (make-wire "down-wire"))

	(cgw-source-atoms sentences 'SentenceNode)
	(wire-probe "sent-list" sentence-parts)
	(cgw-incoming sentence-parts sentences)
)

(define (wire-it-e)
	(define sentences (make-wire "up-wire"))
	(define sentence-parts (make-wire "down-wire"))

	(wire-probe "sent-list" sentence-parts)
	(cgw-incoming sentence-parts sentences)
	(cgw-source-atoms sentences 'SentenceNode)
)

(define (wire-it-f)
	(define sentences (make-wire "up-wire"))
	(define sentence-parts (make-wire "down-wire"))

	(cgw-source-atoms sentences 'SentenceNode)
	(cgw-incoming sentence-parts sentences)
	(wire-probe "sent-list" sentence-parts)
)

;===========================
.
exit
