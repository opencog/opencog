;
; Assorted utilities for supporting face tracking
;
(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog))
(use-modules (opencog exec))
(use-modules (opencog query))

(load-from-path "utilities.scm")

; Is the room empty, or is someone in it?
(define room-state (AnchorNode "Room State"))
(define room-empty (ConceptNode "room empty"))
(define room-nonempty (ConceptNode "room nonempty"))

;; Assume room empty at first
(ListLink room-state room-empty)

; A hacky rule that looks for the visible-face marker, and
; sets the room-is-not-empty flag if a face is visible.
(define chk-room-non-empty
	(BindLink
		(VariableNode "$face-id")
		(EvaluationLink
			(PredicateNode "visible face")
			(ListLink
				(VariableNode "$face-id")
			)
		)
		; Change the status of the room to "non-empty"
		(AssignLink (TypeNode "ListLink") room-state room-nonempty)
	)
)

; A hack rule that inverts the above.
(define chk-room-empty
	(BindLink
		(VariableNode "$face-id")
		(AbsentLink
			(EvaluationLink
				(PredicateNode "visible face")
				(ListLink
					(VariableNode "$face-id")
				)
			)
		)
		; Change the status of the room to "empty"
		(AssignLink (TypeNode "ListLink") room-state room-empty)
	)
)

;; Display the current room state
(define (show-room-state)
	(car (cog-chase-link 'ListLink 'ConceptNode room-state)))


; Quick hack to fill the room.
(define (make-room-nonempty)
	(ListLink room-state (ConceptNode "room nonempty"))
	(cog-delete (ListLink room-state (ConceptNode "room empty")))
)

; Quick hack to clear the room
(define (make-room-empty)
	(ListLink room-state (ConceptNode "room empty"))
	(cog-delete (ListLink room-state (ConceptNode "room nonempty")))
)



#|
;; Example usage:
;;
(cog-bind chk-room-empty)
(cog-bind chk-room-non-empty)
(show-room-state)

(cog-incoming-set (PredicateNode "visible face"))

|#

;; ----
