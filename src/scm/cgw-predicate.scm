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

; 
; Given a stream of atoms
;
(define (cgw-assoc in-wire out-wire link-type in-pos out-pos)

	; Return true iff the atom is of 'link-type'
	(define (is-desired-type? atom)
		(eq? link-type (cog-type atom))
	)

	(define (get-out-atom in-atom)

		; Get the atom in the n-th position of the link
		(define (get-nth link position)
			(list-ref (cog-outgoing-set link) position)
		)

		; Return #t iff the input atom is in location in-pos
		(define (is-in-slot? link)
			; Use equal? not eq? to correctly compare atoms.
			(equal? in-atom (get-nth link in-pos))
		)

		; A list of all links where in-atom in in location in-pos
		(define (good-links)
			(filter is-in-slot? 
				(filter is-desired-type?
					(cog-incoming-set in-atom)
				)
			)
		)

		; Get the out-pos'th atom in the link
		(define (get-out link)
			(get-nth link out-pos)
		)

		; Now get the atoms in the out-pos
		(map get-out (good-links))
	)
	(cgw-transceiver in-wire out-wire get-out-atom get-out-atom)
)


.
exit
