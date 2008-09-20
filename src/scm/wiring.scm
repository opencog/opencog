scm
;
; wiring.scm
;
; Implement a constraint-type wiring system, inspired by SICP sections
; 3.3 (constraints) and 3.5 (streams), where SICP is "Structure and
; Interpretation of Computer Programs", Abelson & Sussman. Note that
; the wiring/constraint paradigm is highly amenable to graphical
; manipulation, along the lines of SGI's widely emulated and copied
; graphical wiring system from the early 1990's.
;
; The core problem that this is attempting to solve is that working
; directly with hypergraphs is hard. Fore example, problems in natural
; language processing require that some certain node in some certain
; location in the hypergraph needs to be compared to some other node
; somewhere else, and if certain conditions apply, then, as a result,
; yet another part of the hypergraph needs to be modified, deleted,
; or constructed; its truth-value needs to be modified, etc.
;
; Code that I've written so far has attacked this problem in an ad-hoc
; manner, crawling the graph, chasing link types in the forward or
; backward direction, looking for nodes of a certain type located in
; a certain place. Worse, this code is fragile: if I decide to change
; how my data (english sentences, grammatical relations, parses) are
; represented in a hypergraph, then I also need to change the code that
; crawls the graph.
;
; There is also another complication: the need for iteration. Whatever
; processing is done, it needs to be done for each word or sentence, etc.
; The ad-hoc proceedure is educational, if you've never used hypergraphs
; before, and relatively fast to implement, but its messy, and certainly
; not scalable. It gets tedious and repetitive -- one codes similar but
; slightly different things with each new algorithm that is needed. Below
; follows an experiment in doing things a different way.
;
; The core paradigm is that of "wires" or "constraints". Wires will
; connect the input or output of one routine to another. Wires carry
; values from one place to another. Consider, for example, the sentence
; "John threw a ball"; it has a relation "_subj(throw, John)" indicating
; that the subject of the verb "throw" is "John". Suppose I wanted to
; iterate over all all relations involving "John" as the subject: that
; is, I want to find all elements ?vrb that solve the predicate
; "_subj(?vrb, John)". This will be done by attaching a wire to ?vrb. The
; result is that the wire will sequentially take on all possible values
; for all matching occrurances of such triples in the hypergraph. This
; wire can be attached to various proceedures that do something with
; those values. Alternately, it might be attached to yet another predicate,
; say "_obj(?vrb, ball)", and would thus hopefully yeild all verbs where
; John did something to a ball.
;
; The design goal is to implement the wires both as constraints (as
; in SICP 3.3) so that they can carry values, and as streams (SICP 3.5)
; so to avoid infinite recursion.
;
; The point of this excercise is not to be abstract, but to explore a
; programming tool. I'm tired of writing ugly, ad-hoc, hard-to-understand
; code for crawling hypergraphs and fetching particular values out of them.
; I'm hoping that this will provide a simpler, cleaner framework.
; The results of this experiment are TBD.
; 
; Some deeper conceptual remarks:
; Of course, a wire should be understood to be a particular type of Link,
; lets say "WireLink".
;
; Implementation notes:
; By analogy to electrical engineering, can also think of wires as "buses";
; values are asserted onto the bus by one master, and all other parties
; on the bus must hold it "floating". Currently, the wire only grants 
; access to one bus-master; its might be possible to relax this.
; May need to create a specific grant/revoke protocal for the bus?

; Create a new wire
; Along the lines of "make-connector", SICP 3.3.5
(define (make-wire)
	(let ((value '())
		(busmaster #f)    ; "informant" in SICP
		(endpoints '()))  ; "constraints" in SICP

		; Connect a new endpoint to the bus.
		; If the endpoint is asserting,
		; propagate the assertion onto the bus
		(define (connect new-endpoint)
			(if (not (memq new-endpoint endpoints))
				(set! endpoints (cons new-endpoint endpoints))
			)

(display "duude enpoint connected\n")
			; XXX If I have a value then inform
			; (if (not (null? value)
			'done
		)

		; Let "master" assert a value onto the bus
		(define (set-value! newval master)
(display "duude set value called\n");
			(cond 
				((null? value) ; if the bus is floating, then grant
					(set! value newval)
					(set! busmaster master)
					(for-each-except master deliver-msg endpoints)
				)
				; XXX if multiple masters, make sure thier values
				; agree, else flag an error
			)
		)

		(define (me request)
			(cond 
				((eq? request 'value) value)
				((eq? request 'has-value?)
					(if busmaster #t #f)
				)
				((eq? request 'connect) connect)
				((eq? request 'set-value!) set-value!)
				(else (error "Unknown operation -- make-wire" request))
			)
		)

		; return the command dispatcher.
		me
	)
)

; return the value of the wire.
(define (wire-get-value wire) (wire 'value))
(define (wire-has-value? wire) (wire 'has-value?))
(define (wire-set-value! wire value endpoint)
	((wire 'set-value!) value endpoint)
)
(define (wire-connect wire endpoint)
	((wire 'connect) endpoint)
)

; Basic messages
; "assert" means "put a message on this bus"
; "float"  means "leave the bus in a floating (unasserted) state"
(define wire-assert-msg 'I-have-a-value)
(define wire-float-msg  'I-lost-my-value)

(define (deliver-msg endpoint) (endpoint wire-assert-msg))

; Display the value on a bus
; Attach an endpoint to the bus, this enpoiint is a read-only endpoint
; It prints any values asserted onto the bus.
(define (wire-probe probe-name wire)
	(define (prt-val value)
		(display "Probe: " )
		(display probe-name)
		(display " = ")
		(display value)
		(newline)
	)

	(define (me request)
		(cond 
			((eq? request wire-assert-msg)
				(prt-val (wire 'value)) 
			)
			((eq? request wire-float-msg)
				(prt-val "floating") 
			)
			(else
				(error "unkonwn request -- wire-probe" request)
			)
		)
	)

	; hook me up
	(wire-connect wire me)

	; return the command dispatcher.
	me
)

; Place a constant value onto a bus
(define (wire-constant value wire)
	(define (me request)
		(error "Unknown request -- wire-constant" request)
	)
	(wire-connect wire me)
	(wire-set-value! wire value me)

	; return the command dispatcher.
	me
)

.
exit
