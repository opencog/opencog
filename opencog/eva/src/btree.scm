;
; btree.scm
;
; Experimental behavior tree in the atomspace.
;

(use-modules (opencog))
(use-modules (opencog query))
(use-modules (opencog exec))


(define (room_empty) (display "ola\n") (stv 1 1))

(DefineLink
	(DefinedPredicateNode "No one visible")
	(EvaluationLink
		(GroundedPredicateNode "scm: room_empty")
		(ListLink)))


(define empty-seq
	(SatisfactionLink
		(SequentialAndLink
			(DefinedPredicateNode "No one visible")
		)))

(cog-satisfy empty-seq)

