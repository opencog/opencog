(use-modules (opencog))
(use-modules (ice-9 threads))

(load "time-map.scm")
(load "self-model.scm")
(load "primitives.scm")

; -----------------------------------------------------------------------------
; Parameters are not published by cmt_tracker and have to be pre-set, so see
; /HEAD/src/vision/cmt_tracker/src/cmt_tracker_node.cpp#L486
; /HEAD/src/vision/pi_vision/pi_face_tracker/nodes/face_tracker.py#L271
; TODO: Move to config file
(define fov 1.42)
(define camera-width 640)
(define camera-height 480)

; `distance` is distance from the camera. The assumption is that at the given
; distance from the camera is the plane where all the faces are at.
; FIXME But why have this plane instead of calculating the distance between
; faces, will it affect the priority?
(define distance 1.0)

; Focal length in pixels
(define k-const (/ camera-width  (tan (/ fov 2.0))))

; The dimenstion of a plane at the given distance.
(define width-of-yz-plane (/ (* camera-width  distance)  k-const))
(define height-of-yz-plane (/ (* camera-height  distance)  k-const))
(define diagonal-of-yz-plane
    (sqrt (+ (expt width-of-yz-plane 2.0) (expt height-of-yz-plane 2.0))))

; Priority scale used for faces.
(define lowest-face-priority  0.0)
(define ordinary-face-priority 0.5)
(define highest-face-priority 1.0)

; -----------------------------------------------------------------------------
(define (get-face-coordinate-in-plane-yz face-id)
    (let ((new-x distance)
          (xyz (get-last-xyz "faces" (Number face-id) face-loc-time-span)))
        (if (null? xyz)
            '()
            (list
                new-x
                (/ (* (list-ref xyz 1) new-x) (list-ref xyz 0))
                (/ (* (list-ref xyz 2) new-x) (list-ref xyz 0)))
        )
    )
)

(define (distance-in-plane-yz face-id-1 face-id-2)
"
  Distance between the two faces in the yz plane.
"
    (let ((coord-1 (get-face-coordinate-in-plane-yz face-id-1))
          (coord-2 (get-face-coordinate-in-plane-yz face-id-2)))
        (if (or (null? coord-1) (null? coord-2))
            (inf)
            (sqrt (+
                (expt (- (list-ref coord-1 1) (list-ref coord-2 1)) 2.0)
                (expt (- (list-ref coord-1 2) (list-ref coord-2 2)) 2.0)))
        )
    )
)

(define (transition-priority face-id-1 face-id-2)
"
  Transtion priority, T(face-id-2|face-id-1), represents if the system is
  looking at face-id-1, then how likely should it be to shift gaze to
  face-id-2.

  A transition priority of 1 means that the system is very likely to shift from
  face-id-1 to face-id-2 , while a transition priority of 0 means that the
  system is very unlikely to shift from face-id-1 to face-id-2.
"
    (let ((d (distance-in-plane-yz face-id-1 face-id-2)))
        ; This check is for (inf). If d is not (inf) and is greater than
        ; the width-of-yz-plane, then something fishy is going on.
        (if (> d  width-of-yz-plane)
            0
            (- 1 (/ d width-of-yz-plane))
        )
    )
)

; -----------------------------------------------------------------------------
; TODO: Make the set, get, & delete of properites into a utility template
; similar ot change-template and timestamp-template, as similar patterns
; will be used for modeling the world. It should also make moving to protoatoms
; simplier when they are ready.

; Usage:
; (cog-evaluate! (Put
;     (DefinedPredicate "Set face priority")
;     (List (Number "12") (Number ".12"))))
(Define
    (DefinedPredicate "Set face priority")
    (Lambda
        (VariableList
            (TypedVariable
                (Variable "face-id")
                (Type "NumberNode"))
            (TypedVariable
                (Variable "priority")
                (Type "NumberNode")))
        (True (State
                (List
                    (Concept "visual priority")
                    (Variable "face-id"))
                (Variable "priority")))))

(define (set-priority! face-id priority)
    (cog-evaluate! (PutLink
        (DefinedPredicate "Set face priority")
        (List (Number face-id) (Number priority))))
)

(Define
    (DefinedPredicate "Set face transition-priority")
    (Lambda
        (VariableList
            (TypedVariable
                (Variable "face-id")
                (Type "NumberNode"))
            (TypedVariable
                (Variable "priority")
                (Type "NumberNode")))
        (True (State
                (List
                    (Concept "transition-priority")
                    (Variable "face-id"))
                (Variable "priority")))))

(define (set-transition-priority! face-id priority)
    (cog-evaluate! (PutLink
        (DefinedPredicate "Set face transition-priority")
        (List (Number face-id) (Number priority))))
)

; -----------------------------------------------------------------------------
; Usage:
; (cog-execute! (Put
;     (DefinedSchema "Get face priority")
;     (Number 1)))
(Define
    (DefinedSchema "Get face priority")
    (Lambda
        (TypedVariable
            (Variable "filter-face-id")
            (Type "NumberNode"))
        (Get
            (VariableList (Variable "face-id") (Variable "priority"))
            (And
                (Identical (Variable "filter-face-id") (Variable "face-id"))
                (State
                    (List
                        (Concept "visual priority")
                        (Variable "face-id"))
                    (Variable "priority"))))
    ))

(define (get-priority! face-id)
    (define result (cog-execute!
                        (PutLink
                            (DefinedSchema "Get face priority")
                            (Number face-id))))
    (if (equal? (Set) result)
        ; FIXME: There should never be an empty set. Perception pipeline
        ; should add a default priority.
        (begin
            (set-priority! face-id ordinary-face-priority)
            ordinary-face-priority)
        (string->number (cog-name (gdar result)))
    )
)

(Define
    (DefinedSchema "Get face transition-priority")
    (Lambda
        (TypedVariable
            (Variable "filter-face-id")
            (Type "NumberNode"))
        (Get
            (VariableList (Variable "face-id") (Variable "priority"))
            (And
                (Identical (Variable "filter-face-id") (Variable "face-id"))
                (State
                    (List
                        (Concept "transition-priority")
                        (Variable "face-id"))
                    (Variable "priority"))))
    ))

(define (get-transition-priority! face-id)
    (define result (cog-execute!
                        (PutLink
                            (DefinedSchema "Get face transition-priority")
                            (Number face-id))))
    (if (equal? (Set) result)
        (begin
            (set-transition-priority! face-id ordinary-face-priority)
            ordinary-face-priority)
        (string->number (cog-name (gdar result)))
    )
)

; -----------------------------------------------------------------------------
; Usage:
; (cog-evaluate! (PutLink
;     (DefinedPredicate "Delete face priority")
;     (Number 1)))
(DefineLink
    (DefinedPredicate "Delete face priority")
    (Lambda
        (TypedVariable
            (Variable "del-face-id")
            (Type "NumberNode"))
        (True (Put
            (Delete
                (State
                    (List
                        (Concept "visual priority")
                        (Variable "face-id"))
                    (Variable "priority")))
            (Get
                (VariableList (Variable "face-id") (Variable "priority"))
                (And
                    (Identical (Variable "del-face-id") (Variable "face-id"))
                    (State
                        (List
                            (Concept "visual priority")
                            (Variable "face-id"))
                        (Variable "priority"))))
        ))))


(define (delete-priority! face-id)
     (cog-evaluate! (PutLink
         (DefinedPredicate "Delete face priority")
         (Number face-id)))
)

(DefineLink
    (DefinedPredicate "Delete face transition-priority")
    (Lambda
        (TypedVariable
            (Variable "del-face-id")
            (Type "NumberNode"))
        (True (Put
            (Delete
                (State
                    (List
                        (Concept "transition-priority")
                        (Variable "face-id"))
                    (Variable "priority")))
            ; The GetLink is structuraly the same as
            ; (DefinedSchema "Get face priority")
            (Get
                (VariableList (Variable "face-id") (Variable "priority"))
                (And
                    (Identical (Variable "del-face-id") (Variable "face-id"))
                    (State
                        (List
                            (Concept "transition-priority")
                            (Variable "face-id"))
                        (Variable "priority"))))
        ))))


(define (delete-transition-priority! face-id)
     (cog-evaluate! (PutLink
         (DefinedPredicate "Delete face transition-priority")
         (Number face-id)))
)

; -----------------------------------------------------------------------------
; The psi-rule Context
(Define
    (DefinedPredicate "Has person being intracted with changed?")
    (SequentialOr
        (Not (Equal
            (GetLink (State prev-interaction-state (Variable "value")))
            (DefinedSchema "Current interaction target")))
    ))

; The psi-rule action
(Define
    (DefinedPredicate "Update face transition-priorities")
    (SequentialAnd
        (True (Put
            (Evaluation
                (GroundedPredicate "scm: calculate-and-set-transition-priority")
                (List (Variable "face-id")))
            (DefinedSchema "Get acknowledged faces")))
        (True (Put
            (DefinedSchema "Set previous interaction value")
            (DefinedSchema "Current interaction target")))
    ))

(define (calculate-and-set-transition-priority face-id-node)
"
  Returns a NumberNode for the transition-priority of the face-id-node.
"
; Checks are not required b/c this function wouldn't be called
; without the context being satisfied
    (let* ((current-face-id (cog-name (gar
           (cog-execute! (DefinedSchema "Current interaction target")))))
           (face-id (string->number (cog-name face-id-node))))
       ; This is for when the room goes empty
        (if (equal? "none" current-face-id)
            (set-transition-priority! face-id 0.5)
            (set-transition-priority! face-id
                (transition-priority face-id (string->number current-face-id)))
        )
    )
)

(Define
    (DefinedSchema "Set previous interaction value")
    (Lambda
        (Variable "value")
        (State prev-interaction-state (Variable "value"))
    ))

; -----------------------------------------------------------------------------
; Only to be used in case more than one face.
(Define
    (DefinedSchema "Select face by priority")
    (ExecutionOutput
        (GroundedSchema "scm: choose-next-face")
        (List)))

(define-public (choose-next-face)
"
  Returns the face-id Node with the highest (* priority transition-priority)
  from among the faces that are not being interacted with.
"
    (define (get-maximum lst)
        ;Get the maximum from a list of numbers.
        (fold (lambda (prev next) (if (> prev next) prev next)) 0 lst))

    (define (weighted-priority x)
        (let ((p (get-priority! x))
              (tp (get-transition-priority! x)))
            (* p tp)
        ))

    (define (faces-not-interacted-with lst)
        ; TODO: Should this be removed when face-recogniztion and who-said rules
        ; are added.
        (let ((interaction-face (gar (cog-execute!
                (DefinedSchema "Current interaction target")))))
            (remove (lambda (x) (equal? interaction-face x)) lst)))

    (let* ((faces (faces-not-interacted-with (show-acked-faces)))
           (face-ids (par-map (lambda (x) (string->number (cog-name x))) faces))
           (wp (par-map (lambda (x) (weighted-priority x)) face-ids))
           (max-wp (get-maximum wp)))
        (car (par-map (lambda (w f) (if (equal? w max-wp) f #f)) wp faces))
    )
)

; psi-action for it is time to change the face being interacted with.
(DefineLink
	(DefinedPredicate "Change interaction target by priority")
	(SequentialAnd
		(True (Put
			(StateLink request-eye-contact-state (VariableNode "$fid"))
			(DefinedSchema "Select face by priority")))

		; Diagnostic print
		(Evaluation (GroundedPredicate "scm: print-msg-face")
			(ListLink (Node "Requested new interaction")))
	))
