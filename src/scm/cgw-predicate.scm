scm
;
; cgw-predicate.scm
;
; Predicate support.
; See cgw-wire.scm for an overview.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
; --------------------------------------------------------------------
;
; cgw-outgoing-nth a-wire b-wire position
;
; Get the n'th atom in the outgoing set.
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
	(wire-transceiver a-wire b-wire get-nth)
)

; --------------------------------------------------------------------
;
; cgw-assoc-uni in-wire out-wire link-type in-pos out-pos
;
; Produce a stream of atoms depending on thier position in a Link.
; Given a stream of atoms in the in-wire, produce a stream of atoms
; on the out-wire, such that a given in-atom is connected to the
; corresponding out-atom by a link of 'link-type', and such that 
; the in-atom appears in position 'in-pos' of the link, and the 
; out-atom appears in position 'out-pos' of the link.
;
; Example:
;   (cgw-assoc in-wire out-wire 'ListLink 0 1)
; will return the atoms in position 1 of all ListLinks where the 
; input atom was in position 0.
;
; See also:
;    cgw-assoc for a bi-directiional equivalant of this routine.
;    cgw-follow-link for similar non-position-dependent function.
;
(define (cgw-assoc-uni in-wire out-wire link-type in-pos out-pos)

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
	(wire-transceiver in-wire out-wire get-out-atom)
)

; --------------------------------------------------------------------
;
; cgw-assoc a-wire b-wire link-type a-pos b-pos
;
; A bidirectional version of cgw-assoc-uni. That is, it functions
; just as cgw-assoc-uni does, but either wire can be the input wire;
; the other wire will be the output wire.
;
; This is implemented by listening for transmit messages. When
; a transmit message is overheard, then that wire is hooked up
; up to the uni-directional version, with the transmitter on the
; input side.
;
(define (cgw-assoc a-wire b-wire link-type a-pos b-pos)
	(let ((device (wire-null-device))
			(do-connect #t)
			(myname "")
		)

		(define (process-msg)
			(cond
				; Input on a-wire, output on b-wire. Disconnect ourself, connect
				; the uni-directional part in the right direction.
				((and (wire-has-stream? a-wire) (not (wire-has-stream? b-wire)))
					(wire-disconnect a-wire me)
					(wire-disconnect b-wire me)
					(set! do-connect #f)
					(set! device (cgw-assoc-uni a-wire b-wire link-type a-pos b-pos))
				)

				; Input on b-wire, output on a-wire. Disconnect ourself, connect
				; the uni-directional part in the right direction.
				((and (not (wire-has-stream? a-wire)) (wire-has-stream? b-wire))
					(wire-disconnect a-wire me)
					(wire-disconnect b-wire me)
					(set! do-connect #f)
					(set! device (cgw-assoc-uni b-wire a-wire link-type b-pos a-pos))
				)

				; Both wires floating. Disconnect the device, if previously attached,
				; and re-connect ourselves, so as to hear about anything new.
				((and (not (wire-has-stream? a-wire)) (not (wire-has-stream? b-wire)))
					(wire-disconnect a-wire device)
					(wire-disconnect b-wire device)
					(set! device (wire-null-device))
					(wire-connect a-wire me)
					(wire-connect b-wire me)
		(display "duuude both floating !\n")
				)

				; Error condition
				((and (wire-has-stream? a-wire) (wire-has-stream? b-wire))
					(error "Both wires have streams! -- cgw-assoc")
				)
			)
		)

		(define (me msg)
			(cond
				((eq? msg wire-assert-msg)
					(process-msg)
				)
				((eq? msg wire-float-msg)
					;; Ignore the float message
				)
				(else
					(default-dispatcher msg 'cgw-assoc myname)
 				)
			)
		)

		; Handy debugging prints
		; (set! myname "being-manually-debugged")
		; (wire-set-name a-wire "cgw-assoc a-wire")
		; (wire-set-name b-wire "cgw-assoc b-wire")
		; (wire-enable-debug a-wire)
		; (wire-enable-debug b-wire)
	
		; Two distinct checks of 'do-connect' are made, because the
		; act of connecting the a-wire could trigger a message that
		; hooks up the actual processor (and thus should leave 
		; wire-b not connected here)
		(if do-connect
			(wire-connect a-wire me)
		)
		(if do-connect
			(wire-connect b-wire me)
		)
	)
)

; --------------------------------------------------------------------
;
; Link splitter
; Given a stream of links on the 'in-link-wire', generate two streams
; of atoms, with the first stream, on the 'a-out-wire', consisting of
; atoms occuppying position 'a-pos' in the link, and the 'b-out-wire'
; getting the atoms in position 'b-pos' of the same link.
;
; Here, 'a-pos' and 'b-pos' are integers, denoting positions in the 
; outgoing-set of a link, starting with position 0.
;
(define (cgw-splitter in-link-wire a-out-wire b-out-wire a-pos b-pos)

	(define aw (make-wire))
	(define bw (make-wire))
	(wire-fan-out in-link-wire aw bw)
	(cgw-outgoing-nth aw a-out-wire a-pos)
	(cgw-outgoing-nth bw b-out-wire b-pos)
)

; --------------------------------------------------------------------
;
; Generalized predicate
;
(define (cgw-predicate wire-a wire-b wire-c)

	(cgw-triplet wire-a wire-b wire-c 'EvaluationLink 'ListLink)
)

(define (cgw-triplet wire-a wire-b wire-c link-hi link-lo)

	(define lopair (make-wire))
	(define lotype (make-wire))

	(cgw-assoc wire-a lopair link-hi 0 1)
	(cgw-filter-atom-type lopair lotype)

	(wire-fan-out lotype wire-b wire-c)

)
.
exit
