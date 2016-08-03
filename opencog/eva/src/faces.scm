;
; Assorted utilities for supporting face tracking
; XXX most of face-tracking is now in self-model.scm
; Perhaps this file is not needed any more? XXX FIXME
;
(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog))
(use-modules (opencog ato pointmem))
(use-modules (opencog exec))
(use-modules (opencog query))

;; XXX FIXME: This file defines a "Room State", which currently can
;; be "empty" or "non-empty", depending on whether faces are visible
;; or not.  But this is kind-of pointless: its probably easier to just
;; check if the number of visible faces is greater than zero, or not.
;; Thus, this adds a complex mechanism, ripe for bugs, that is not
;; really needed .. and should probably be removed.  Right?

; Is the room empty, or is someone in it?
(define room-state (AnchorNode "Room State"))
(define room-empty (ConceptNode "room empty"))
(define room-nonempty (ConceptNode "room nonempty"))

;; Assume room empty at first
(StateLink room-state room-empty)

; A rule that looks for the visible-face marker, and
; sets the room-is-not-empty flag if a face is visible.
(DefineLink
	(DefinedPredicateNode "Check if room non-empty")
	(SatisfactionLink
		(SequentialAndLink
			; If someone is visible...
			(PresentLink (EvaluationLink (PredicateNode "visible face")
					(ListLink (VariableNode "$face-id"))))
			; Change the status of the room to "non-empty"
			(TrueLink (PutLink
					(StateLink room-state (VariableNode "$x"))
					room-nonempty)))))

; A rule that inverts the above.
(DefineLink
	(DefinedPredicateNode "Check if room empty")
	(SatisfactionLink
		(SequentialAndLink
			; If no-one is visible...
			(AbsentLink (EvaluationLink (PredicateNode "visible face")
						(ListLink (VariableNode "$face-id"))))

			; Change the status of the room to "empty"
			(TrueLink (PutLink
					(StateLink room-state (VariableNode "$x"))
					room-empty)))))

; A rule to update the room state
(DefineLink
	(DefinedPredicateNode "Update room state")
	(SatisfactionLink
		(SequentialOrLink
			(DefinedPredicateNode "Check if room non-empty")
			(DefinedPredicateNode "Check if room empty"))))

; -----------------------------------------------------------------
; Assorted debugging utilities.
;
;; Display the current room state
(define-public (show-visible-faces)
	(define visible-face (PredicateNode "visible face"))
	(filter (lambda(y) (equal? (cog-type y) 'NumberNode))
	(map (lambda (x) (car (cog-outgoing-set x)))
	(cog-chase-link 'EvaluationLink 'ListLink visible-face))))

(define-public (show-acked-faces)
	(define acked-face (PredicateNode "acked face"))
	(filter (lambda(y) (equal? (cog-type y) 'ConceptNode))
	(map (lambda (x) (car (cog-outgoing-set x)))
	(cog-chase-link 'EvaluationLink 'ListLink acked-face))))

(define-public (show-recognized-faces)
"
 Show face-id recognized-face name pairs in atomese
"
	(cog-outgoing-set (cog-execute! (DefinedSchema "Get recognized faces")))
)

(define-public (show-room-state)
	(car (cog-chase-link 'StateLink 'ConceptNode room-state)))


(define-public (show-eye-contact-state)
	(define e-c-state (Anchor "Eye Contact State"))
	(car (cog-chase-link 'StateLink 'NumberNode e-c-state)))


; define-public because `unit-test.scm` uses it.
(define-public (make-new-face id)
"
 make-new-face ID

 Debug utility - Quick hack to fill the room.

 Call this function to trick opencog into thinking there is a new
 visible face.  There will not be any corresponding 3D coords, so
 the ROS tf2 will not be able to make the robot turn to look...
"
	(EvaluationLink (PredicateNode "visible face")
		(ListLink (NumberNode id))))

(define-public (make-mapped-face face-id x y z)
"
  make-mapped-face FACE-ID X Y Z

  FACE-ID is a number that represents the face ID, and RECOG-ID is a number that
  represents the recognition ID. X, Y and Z are the coordinate numbers
  associated with the face that represented by FACE-ID. It returns the atomese
  representation of a visible face that was added to the octomap.
"
	(map-ato "faces" (NumberNode face-id (av 5 0 0)) x y z)
	(make-new-face (number->string face-id))
)

(define-public (make-recognized-face face-id recog-id)
"
  make-recognized-face FACE-ID RECOG-ID

  FACE-ID is a number that represents the face ID and RECOG-ID is a string that
  represents the recognition ID for the visible face. It returns the atomese
  representation of the recognized face.
"
	(EvaluationLink
		(PredicateNode "name")
		(ListLink
			(ConceptNode (number->string face-id))
			(ConceptNode recog-id)))
)

(define-public (make-mapped-recognized-face face-id recog-id x y z)
"
  make-mapped-recognized-face FACE-ID RECOG-ID X Y Z

  FACE-ID is a number that represents the face ID, and RECOG-ID is a string that
  represents the recognition ID. X, Y and Z are the coordinate numbers
  associated with the face that represented by FACE-ID. It returns the atomese
  representation of the recognized face that was added to the octomap.

  If RECOG-ID is `0` then it is an unrecognized face.
"
	(make-mapped-face face-id x y z)
	(make-recognized-face face-id recog-id)
)

(define-public (remove-face id)
"
 remove-face ID

 Quick hack to remove face ID from the room
"
	(cog-delete (EvaluationLink (PredicateNode "visible face")
		(ListLink (ConceptNode id)))))

(define (undefine def)
	(cog-delete (car (cog-incoming-set def))))

#|
;; Example usage:
;;
(cog-evaluate! (DefinedPredicateNode "Update room state"))
(show-room-state)

(cog-incoming-set (PredicateNode "visible face"))

|#

;; ----
