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
	(wire-transceiver a-wire b-wire show)
)

;; -------------------------------------------------------------------------
;
; cgw-incoming a-wire b-wire
;
; Given a stream of atoms on a wire, the incoming set of those atoms
; are presented on the other wire.
;
(define (cgw-incoming a-wire b-wire)
	(wire-transceiver a-wire b-wire cog-incoming-set)
)

; cgw-outgoing a-wire b-wire
;
; Given a stream of atoms on a wire, the outgoing set of those atoms
; are presented on the other wire.
;
(define (cgw-outgoing a-wire b-wire)
	(wire-transceiver a-wire b-wire cog-outgoing-set)
)

;; -------------------------------------------------------------------------
;
; cgw-filter-atom-type a-wire b-wire atom-type
;
; Filter based on atom types. Is the atoms presenting on the wires are 
; of the given type, then the atoms can pass through, else they are discarded.
; This filter is bi-directional (works in either direction).
;
(define (cgw-filter-atom-type a-wire b-wire atom-type)
	(wire-filter a-wire b-wire (lambda (atom) (eq? atom-type (cog-type atom))))
)

;; --------------------------------------------------------------------
;
; cgw-filter-arity
;
; Filter a stream based on whether it's outgoing set has at least n items.
; That is, an atom, presenting on input, must be a link, just in order to 
; have an outgoing set, and it must have an arity of at least n.
;
(define (cgw-filter-arity a-wire b-wire arity)
	(define (has-arity? link)
		(<= arity (cog-arity link))
	)
	(wire-filter a-wire b-wire has-arity?)
)

;; --------------------------------------------------------------------
;
; cgw-filter-incoming input-wire output-wire link-type
;
; Get the incoming-set of the atoms on the input-wire, and filter it
; by link-type, presenting the results on the output-wire. Of course,
; for any given atom, its incoming set consists entirely of links.
; This filter is uni-directional, with a stated input and output.
;
; See also:
;    cgw-filter-incoming-pos for a routine that also requires the 
;    input atom to appear in a specific position in the link.
;
(define (cgw-filter-incoming input-wire output-wire link-type)
	(define mid (make-wire))
	(cgw-incoming input-wire mid)
	(cgw-filter-atom-type mid output-wire link-type)
)

;; --------------------------------------------------------------------
;
; cgw-filter-outgoing input-wire output-wire atom-type
;
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

;; --------------------------------------------------------------------
;
; cgw-follow-link input-wire output-wire link-type target-type
;
; Follow a link to a target atom type. 
; Starting with the atom on input-wire, examine its incoming-set,
; selecting only those with 'link-type'. Then examine the 
; outgoing-set of the link, selecting only those atoms of 
; 'target-type'. Place those atoms on the output-wire. 
;
(define (cgw-follow-link input-wire output-wire link-type target-type)
	(define mid (make-wire))
	(cgw-filter-incoming input-wire mid link-type)
	(cgw-filter-outgoing mid output-wire target-type)
)

;; --------------------------------------------------------------------
;
; cgw-follow-link-pos input-wire output-wire link-type in-pos out-pos
;
; Follow a link to an atom located in a certain position.
; Starting with the atom on input-wire, examine its incoming-set,
; selecting only those with type 'link-type', and where the incoming
; atom appears in position 'in-pos'. Place on the output wire those
; atoms which appear in the 'out-pos' slot of the link.
;
; See also:
; This is equivalent to using cgw-binary-link, and leaving the
; link-wire disconnected.
;
(define (cgw-follow-link-pos input-wire output-wire link-type in-pos out-pos)
	(define mid (make-wire))
	(cgw-filter-incoming-pos-uni input-wire mid link-type in-pos)
	(cgw-outgoing-nth mid output-wire out-pos)
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
	(define (filter elt)
		(if (predicate elt)
			(list elt)
			'()
		)
	)
	(wire-transceiver A-wire B-wire filter)
)

;; -------------------------------------------------------------------------
;
; wire-transceiver a-wire b-wire xform-proc
;
; Generic transceiver component for transforming data between two wires.
; This component applies the proceedure 'proc' to all data flowing
; between the two wires (in either direction). One wire (either wire)
; is taken as input, the other will then present the output. 
;
; The procedure "proc" is called on each stream element. It should take
; a single element as input, and produce a list of elements as output;
; it may produce a null list, or a list with one or more elements.
;
; Typically, the stream elements are presumed to be opencog atoms;
; however, nothing in the implementation presumes this. In particular,
; the transceiver *could* be used to perform transformations on truth
; values; alternately, it could be used to convert a stream of atoms
; into a stream of truth values, and so on.
;
(define (wire-transceiver up-wire down-wire xform-proc)

	(let ((input-stream stream-null)
			(myname "")
		)

		; Define a generic producer function for a stream. This producer 
		; pulls elements off the input stream, and applies the function 
		; 'xform-func'. This function should produce a list of elements; 
		; these are then posted.
		(define (producer state xform-func)
			; If we are here, we're being forced.
			(if (null? state)
				(if (stream-null? input-stream)
					#f ; we are done, the stream has been drained dry
					; else grab an element from the input-stream
					(let ((atom (stream-car input-stream)))
						(set! input-stream (stream-cdr input-stream))
						(if (null? atom)
							(error "Unexpected empty stream! wire-transceiver")
							(producer (xform-func atom) xform-func)
						)
					)
				)
				; else state is a list of elements, so keep letting 'er rip.
				state
			)
		)

		; create a stream, useing the producer function
		(define (mkstrm)
			(make-stream (lambda (state) (producer state xform-proc)) '())
			; (list->stream (list 'a 'b 'c))  ; sample test gen
		)

		(define (hookup in-wire out-wire)
			(set! input-stream (wire-take-stream in-wire))
			(wire-set-stream! out-wire (mkstrm) me)
		)

		(define (process-msg)
			(cond
				((and (wire-has-stream? up-wire) (not (wire-has-stream? down-wire)))
					; If we are here, there's a stream on the up-wire. 
					; transform it and send it.
					;; but first, make sure the down wire is connected!
					(wire-connect down-wire me)
					(hookup up-wire down-wire)
				)

				((and (not (wire-has-stream? up-wire)) (wire-has-stream? down-wire))
					; If we are here, there's a stream on the down-wire. 
					; transform it and send it.
					;; but first, make sure the up wire is connected!
					(wire-connect up-wire me)
					(hookup down-wire up-wire)
				)

				((and (wire-has-stream? up-wire) (wire-has-stream? down-wire))
					(error "Both wires have streams! -- wire-transceiver")
				)
			)
		)

		; Message dispatcher
		(define (me msg)
			(cond 
				((eq? msg wire-assert-msg)
					(process-msg)
				)
				
				((eq? msg wire-float-msg)
					;; Ignore the float message
					;; XXX unclear -- perhaps we should set the input stream
					;; to null, if both wires are floating? e.g. if they are
					;; later disconnected?
				)
				(else 
					(default-dispatcher msg 'wire-transceiver myname)
				)
			)
		)

		;; connect the wires, if not already done so
		(wire-connect up-wire me)
		(wire-connect down-wire me)
		me
	)
)

;; -------------------------------------------------------------------------

.
exit
