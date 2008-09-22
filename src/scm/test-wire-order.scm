scm
;===========================
; wire-order tests
; test that the results of wiring are independent of the order 
; of the operations.

(define (wire-it-a)
	(define sentences (make-wire))
	(define sentence-parts (make-wire))

	(cgw-xfer sentences sentence-parts)
	(wire-probe "sent-list" sentence-parts)
	(cgw-source-atoms sentences 'SentenceNode)
)

(define (wire-it-b)
	(define sentences (make-wire))
	(define sentence-parts (make-wire))

	(cgw-xfer sentences sentence-parts)
	(cgw-source-atoms sentences 'SentenceNode)
	(wire-probe "sent-list" sentence-parts)
)

(define (wire-it-c)
	(define sentences (make-wire))
	(define sentence-parts (make-wire))

	(wire-probe "sent-list" sentence-parts)
	(cgw-source-atoms sentences 'SentenceNode)
	(cgw-xfer sentences sentence-parts)
)

(define (wire-it-d)
	(define sentences (make-wire))
	(define sentence-parts (make-wire))

	(cgw-source-atoms sentences 'SentenceNode)
	(wire-probe "sent-list" sentence-parts)
	(cgw-xfer sentences sentence-parts)
)

(define (wire-it-e)
	(define sentences (make-wire))
	(define sentence-parts (make-wire))

	(wire-probe "sent-list" sentence-parts)
	(cgw-xfer sentences sentence-parts)
	(cgw-source-atoms sentences 'SentenceNode)
)

(define (wire-it-f)
	(define sentences (make-wire))
	(define sentence-parts (make-wire))

	(cgw-source-atoms sentences 'SentenceNode)
	(cgw-xfer sentences sentence-parts)
	(wire-probe "sent-list" sentence-parts)
)

;===========================
.
exit
