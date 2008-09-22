scm
;
; cgw-predicate.scm
;
; Predicate support.
; See cgw-wire.scm for an overview.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
;

; cgw-outgoing-nth a-wire b-wire position
;
; Get the n'th atom in the ougoing set.
; Given a stream of atoms on either of the wires, the n'th
; atom from the outgoing set is passed to the other wire.
; Counting starts from zero: a link with two atoms has those
; atoms in positions 0, 1.
;
(define (cgw-outgoing-nth a-wire b-wire position)

	(define (get-nth link)
		(let ((atom (list-ref (cog-outgoing-set link) position)))
			(if (null? atom)
				'()
				(list atom)
			)
		)
	)
	(cgw-transceiver a-wire b-wire get-nth get-nth)
)

; (define (cgw-is-nth a-wire b-wire position)


.
exit
