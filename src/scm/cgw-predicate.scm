scm
;
; cgw-predicate.scm
;
; Predicate support.
; See WIRING-README for an overview.
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
		(if (>= position (cog-arity link))
			'()
			(list (list-ref (cog-outgoing-set link) position))
		)
	)
	(wire-transceiver a-wire b-wire get-nth)
)

; --------------------------------------------------------------------
;
; cgw-filter-incoming-pos-uni in-wire out-wire link-type in-pos out-pos
;
; Produce links that contains the given atoms in the n-th position. 
; Given a stream of atoms on the in-wire, produce a stream of links
; on the out-wire, such that a given in-atom appears in position
; 'atom-pos' of the link, and the link is of 'link-type'.
;
; Example:
;   (cgw-filter-incoming-pos-uni in-wire out-wire 'ListLink 1)
; will return the all links of type "ListLink" where the input atom
; was in position 1.
;
; See also:
;    cgw-filter-incoming-pos for a bi-directiional equivalant of this routine.
;    cgw-filter-incoming for a similar non-position-dependent function.
;
(define (cgw-filter-incoming-pos-uni in-wire out-wire link-type atom-pos)

	; Return true iff the atom is of 'link-type'
	(define (is-desired-type? atom)
		(eq? link-type (cog-type atom))
	)

	; Get the atom in the n-th position of the link
	(define (get-nth link position)
		(list-ref (cog-outgoing-set link) position)
	)

	(define (good-links in-atom)
		; Return #t iff the input atom is in location in-pos
		(define (is-in-slot? link)
			; Use equal? not eq? to correctly compare atoms.
			(equal? in-atom (get-nth link atom-pos))
		)

		; A list of all links where in-atom in in location in-pos
		(filter is-in-slot? 
			(filter is-desired-type?
				(cog-incoming-set in-atom)
			)
		)
	)
	(wire-transceiver in-wire out-wire good-links)
)

; --------------------------------------------------------------------
;
; cgw-filter-incoming-pos a-wire b-wire link-type a-pos b-pos
;
; A bidirectional version of cgw-filter-incoming-pos-uni. That is, it
; functions just as cgw-filter-incoming-pos-uni does, but either wire
; can be the input wire; the other wire will be the output wire.
;
(define (cgw-filter-incoming-pos a-wire b-wire link-type a-pos)
	(wire-bidi a-wire b-wire
		(lambda (aw bw) (cgw-filter-incoming-pos-uni aw bw link-type a-pos))
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
(define (cgw-splitter link-wire a-wire b-wire link-type a-pos b-pos)
	(let (
			(l-device (wire-null-device))
			(a-device (wire-null-device))
			(b-device (wire-null-device))
			(do-connect #t)
			(myname "")
		)

		(define (process-msg)
			;;
			;; The message is almost surely telling us to change the wiring;
			;; and so disconnect whatever we were previously attached to.
			(wire-disconnect a-wire a-device)
			(wire-disconnect b-wire b-device)
			(wire-disconnect link-wire l-device)
			(set! do-connect #f)
			(cond
				;; input on link-wire only
				((and (wire-has-stream? link-wire)
						(not (wire-has-stream? a-wire))
						(not (wire-has-stream? b-wire))
					)
					(let ((aw (make-wire))
							(bw (make-wire))
							(lw (make-wire))
						)
						(set! l-device (cgw-filter-atom-type link-wire lw link-type))
						(wire-fan-out lw aw bw)
						(set! a-device (cgw-outgoing-nth aw a-wire a-pos))
						(set! b-device (cgw-outgoing-nth bw b-wire b-pos))
					)
				)

				;; input on a-wire only
				((and (not (wire-has-stream? link-wire))
						(wire-has-stream? a-wire)
						(not (wire-has-stream? b-wire))
					)
					(let ((lw (make-wire))
							(bw (make-wire))
							(ar (make-wire))
						)
						(set! a-device (cgw-filter-incoming-pos-uni a-wire lw link-type a-pos))
						(wire-fan-out lw ar bw)
						;; The arity filter prevents us from returning links
						;; which can't have a b-pos.
						(set! l-device (cgw-filter-arity ar link-wire (+ b-pos 1)))
						(set! b-device (cgw-outgoing-nth bw b-wire b-pos))
					)
				)

				;; input on b-wire only
				((and (not (wire-has-stream? link-wire))
						(not (wire-has-stream? a-wire))
						(wire-has-stream? b-wire)
					)
					(let ((lw (make-wire))
							(aw (make-wire))
							(ar (make-wire))
						)
						(set! b-device (cgw-filter-incoming-pos-uni b-wire lw link-type b-pos))
						(wire-fan-out lw ar aw)
						;; The arity filter prevents us from returning links
						;; which can't have an a-pos.
						(set! l-device (cgw-filter-arity ar link-wire (+ a-pos 1)))
						(set! a-device (cgw-outgoing-nth aw a-wire a-pos))
					)
				)
			)
		)

		(define (me msg)
			(cond
				((eq? msg wire-assert-msg)
					(process-msg)
				)
				((eq? msg wire-float-msg)
					;; XXX ignore for now ... 
				)
				(else
					(default-dispatcher msg 'cgw-splitter myname)
 				)
			)
		)

		; Three distinct checks of 'do-connect' are made, because the
		; act of connecting the wires could trigger a message that
		; hooks up the actual processor.
		(if do-connect
			(wire-connect a-wire me)
			(set! a-device me)
		)
		(if do-connect
			(wire-connect b-wire me)
			(set! b-device me)
		)
		(if do-connect
			(wire-connect link-wire me)
			(set! l-device me)
		)
		me
	)
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

	(cgw-filter-incoming-pos wire-a lopair link-hi 0 1)
	(cgw-filter-atom-type lopair lotype)

	(wire-fan-out lotype wire-b wire-c)

)
.
exit
