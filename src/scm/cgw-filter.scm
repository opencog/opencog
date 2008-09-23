scm
;
; cgw-filter.scm
;
; Basic two-wire filtering and transformation components.
; See cgw-wire.scm for an overview.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;

;; -------------------------------------------------------------------------
; 
; Place all atoms in the atomspace, of type 'atom-type', onto wire
(define (cgw-source-atoms wire atom-type)
	(wire-source-list wire (cog-get-atoms atom-type))
)

;; -------------------------------------------------------------------------
;
; Print types of atoms on a wire
;
(define (cgw-display-atom-type a-wire b-wire)
	(define (show atom)
		(display "Atom type: ")
		(display (cog-type atom))
		(newline)
		(list atom)
	)
	(wire-transceiver a-wire b-wire show show)
)

;; -------------------------------------------------------------------------
;
; cgw-incoming a-wire b-wire
;
; Given a stream of atoms on a wire, the incoming set of those atoms
; are presented on the other wire.
;
(define (cgw-incoming a-wire b-wire)
	(wire-transceiver a-wire b-wire cog-incoming-set cog-incoming-set)
)

; cgw-outgoing a-wire b-wire
;
; Given a stream of atoms on a wire, the outgoing set of those atoms
; are presented on the other wire.
;
(define (cgw-outgoing a-wire b-wire)
	(wire-transceiver a-wire b-wire cog-outgoing-set cog-outgoing-set)
)

; cgw-xfer up-wire down-wire
;
; Transform an atom to its incoming/outgoing list
; For every atom placed on the up-wire, that atom's outgoing set will
; be placed on the down-wire.  For every atom placed on the down-wire,
; that atom's incoming set will be placed in the up-wire.
;
; Note, this routine is dangerous, in that its easy to make mistakes
; about which wire is which.  Suggest using the simpler, less error
; print routines 'cgw-incoming' or 'cgw-outoing'.
;
(define (cgw-xfer up-wire down-wire)
	(wire-transceiver up-wire down-wire cog-outgoing-set cog-incoming-set)
)

;; -------------------------------------------------------------------------
;
; Filter based on atom types. Is the atoms presenting on the wires are 
; of the given type, then the atoms can pass through, else they are discarded.
; This filter is bi-directional (works in either direction).
;
(define (cgw-filter-atom-type a-wire b-wire atom-type)
	(wire-filter a-wire b-wire (lambda (atom) (eq? atom-type (cog-type atom))))
)

; Get the incoming-set of the atoms on the input-wire, and filter it
; by atom-type presenting the results on the output-wire.
; This filter is uni-directional, with a stated input and output.
;
(define (cgw-filter-incoming input-wire output-wire atom-type)
	(define mid (make-wire))
	(cgw-incoming input-wire mid)
	(cgw-filter-atom-type mid output-wire atom-type)
)

; Get the outgoing-set of the atoms on the input-wire, and filter it
; by atom-type presenting the results on the output-wire. Of course,
; only links have outgoing sets, so any nodes on the input-wire will
; get discarded.
;
(define (cgw-filter-outgoing input-wire output-wire atom-type)
	(define mid (make-wire))
	(cgw-outgoing input-wire mid)
	(cgw-filter-atom-type mid output-wire atom-type)
)

; cgw-follow-link input-wire output-wire link-type target-type
;
; Follow a link to a target atom type. 
; Starting with the atom on input-wire, examine its incoming-set,
; selecting only those with 'link-type'. Then examine the 
; outgoing-set of the link, selecting only those atoms of 
; 'target-type'. Place those atoms on the output-wire. 
;
; See also:
;    cgw-assoc for a similar, position-dependent link-chase
;
(define (cgw-follow-link input-wire output-wire link-type target-type)
	(define mid (make-wire))
	(cgw-filter-incoming input-wire mid link-type)
	(cgw-filter-outgoing mid output-wire target-type)
)

;; -------------------------------------------------------------------------
;
; Passive filter -- block everything on the wire that doesn't satisfy
; the predicate. That is, if there is a stream of elements on either
; the A or B wires, then apply the predicate to each element. If
; the predicate returns true, then allow the element to pass to the
; other wire, else discard the element.
;
(define (wire-filter A-wire B-wire predicate)
	(define (filter atom)
		(if (predicate atom)
			(list atom)
			'()
		)
	)
	(wire-transceiver A-wire B-wire filter filter)
)

;; -------------------------------------------------------------------------
;
; wire-transceiver up-wire down-wire up-to-down-proc down-to-up-proc
;
; Generic transceiver component for transforming data between two wires.
; This component attaches between two wires, called "up" and "down".
; If there is a stream on the up-wire, then that stream is taken, and 
; the "up-to-down-proc" is called on the stream elements, and a new 
; stream is generated on the down-wire, which contains the results of
; applying the "up-to-down-proc" on the stream elements.  it can also 
; work in the opposire direction, so that if it is the down-wire that
; has a producer on it, then the "down-to-up-proc" is called, and the
; results are posted on the up-wire.
;
; The two procs must be able to take elements of the steam, and must
; produce lists of elements suitable for the stream (including the null
; list). Typically, the stream elements are presumed to be opencog atoms;
; however, nothing in the implementation presumes this. In particular,
; the transceiver *could* be used to perform transformations on truth
; values; alternately, it could be used to convert a stream of atoms
; into a stream of truth values.
;
; The only requirement is that the up-to-down-proc must accept as
; input the elements of the up-wire, and must produce lists of elements
; appropriate for the down-wire.
;
(define (wire-transceiver up-wire down-wire up-to-down-proc down-to-up-proc)

	(let ( (input-stream stream-null) )

		; Define a generic producer function for a stream. This producer 
		; pulls elements off the input stream, and applies the function 
		; 'cog-func'. This function should produce a list of elements; 
		; these are then posted.
		(define (producer state cog-func)
			; If we are here, we're being forced.
			(if (null? state)
				(if (stream-null? input-stream)
					#f ; we are done, the stream has been drained dry
					; else grab an element from the input-stream
					(let ((atom (stream-car input-stream)))
						(set! input-stream (stream-cdr input-stream))
						(if (null? atom)
							(error "Unexpected empty stream! wire-transceiver")
							(producer (cog-func atom) cog-func)
						)
					)
				)
				; else state is a list of elements, so keep letting 'er rip.
				state
			)
		)

		; Pull a stream off the wire, and create a new stream which will
		; apply 'proc' to the input-stream to generate the output stream.
		(define (make-wire-stream wire proc)
			(set! input-stream (wire-take-stream wire))
			(if (stream-null? input-stream)
				(error "Impossible condition: wire has no stream! -- wire-transciever")
			)
			; (list->stream (list 'a 'b 'c))  ; sample test gen
			(make-stream proc '())
		)

		(define (pull-from-up state)
			(producer state up-to-down-proc)
		)

		(define (pull-from-down state)
			(producer state down-to-up-proc)
		)

		(define (up-me msg)
			(cond 
				((eq? msg wire-assert-msg)
					; If we are here, there's a stream on the up-wire. 
					; transform it and send it.
					;; but first, make sure the down wire is connected!
					(wire-connect down-wire down-me)
					(wire-set-stream! down-wire (make-wire-stream up-wire pull-from-up) down-me)
				)
				
				((eq? msg wire-float-msg)
					;; Ignore the float message
				)
				(else (error "Unknown message -- wire-transceiver up-wire"))
			)
		)

		(define (down-me msg)
			(cond
				((eq? msg wire-assert-msg)
					; If we are here, there's a stream on the down-wire. 
					; transform it and send it.
					;; but first, make sure the up wire is connected!
					(wire-connect up-wire up-me)
					(wire-set-stream! up-wire (make-wire-stream down-wire pull-from-down) up-me)
				)
				((eq? msg wire-float-msg)
					;; Ignore the float message
				)
				(else (error "Unknown message -- wire-transceiver down-wire"))
			)
		)

		;; connect the wires, if not already done so
		(wire-connect up-wire up-me)
		(wire-connect down-wire down-me)
	)

	'()
)

;; -------------------------------------------------------------------------

.
exit
