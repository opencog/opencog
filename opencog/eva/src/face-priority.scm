(use-modules (opencog))
(use-modules (ice-9 threads))
(use-modules (opencog eva-model) (opencog eva-behavior))

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
"
  Returns (stv 1 1) after setting the visual-priority property for the face
  with id equaling face-id.
"
    (State
        (List
            (Concept "visual priority")
            (Number face-id))
        (Number priority))

    (stv 1 1)
)

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
            (VariableList
                (TypedVariable (Variable "face-id") (Type "NumberNode"))
                (TypedVariable (Variable "priority") (Type "NumberNode")))
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
        ; FIXME: There should never be an empty set. The value should be set
        ; during acknowledgment.
        (begin
            (set-priority! face-id ordinary-face-priority)
            ordinary-face-priority)
        (string->number (cog-name (gdar result)))
    )
)

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
                (VariableList
                    (TypedVariable (Variable "face-id") (Type "NumberNode"))
                    (TypedVariable (Variable "priority") (Type "NumberNode")))
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

;; Convert from at location link to (x y z) list
(define (space-nodes at-loc-link)
   (cog-outgoing-set (cadr
      (cog-outgoing-set (cadr (cog-outgoing-set at-loc-link))))))

(define (loc-link-x at-loc-link)
   (string->number (cog-name (car (space-nodes at-loc-link))))
)

(define (loc-link-y at-loc-link)
   (string->number (cog-name (cadr (space-nodes at-loc-link))))
)

(define (loc-link-z at-loc-link)
   (string->number (cog-name (caddr (space-nodes at-loc-link))))
)

(define (get-last-xyz map-name id-node elapse)
	(let* ((loc-atom (gar (get-last-locs-ato map-name id-node elapse))))
		(if (not (null? loc-atom))
			(let* ((xx (loc-link-x loc-atom))
				    (yy (loc-link-y loc-atom))
				    (zz (loc-link-z loc-atom)))
				(list xx yy zz))
			(list)
		)
	)
)

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

;;
;; XXX refactor all of this -- this si supposed to be a service provided
;; byt the spoace-time server, instead of being in scheme spaghetti
;; code.
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
            ; FIXME: Sometimes d is larger than the width-of-yz-plane
            ; and returning 0 is causing a problem in 'choose-next-face'
            ; Recalculate the width-of-yz-plane?
            0.0001
            (- 1 (/ d width-of-yz-plane))
        )
    )
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
"
  Returns (stv 1 1) after setting the transition-priority property for the face
  with id equaling face-id.
"
    (State
        (List
            (Concept "transition-priority")
            (Number face-id))
        (Number priority))
    (stv 1 1)
)

(Define
    (DefinedSchema "Get face transition-priority")
    (Lambda
        (TypedVariable
            (Variable "filter-face-id")
            (Type "NumberNode"))
        (Get
            (VariableList
                (TypedVariable (Variable "face-id") (Type "NumberNode"))
                (TypedVariable (Variable "priority") (Type "NumberNode")))
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
    ; As long as the state updating psi-rule is executed, result shouldn't be
    ; (SetLink)
    (string->number (cog-name (gdar result)))
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
                (VariableList
                    (TypedVariable (Variable "face-id") (Type "NumberNode"))
                    (TypedVariable (Variable "priority") (Type "NumberNode")))
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

; The psi-rule action
(Define
    (DefinedPredicate "Update face transition-priorities")
    (SequentialAnd
        (True (Put
            (Evaluation
                (GroundedPredicate "scm: calculate-and-set-transition-priority")
                (List (Variable "face-id")))
            (DefinedSchema "Get acknowledged faces")))
    ))

(define (calculate-and-set-transition-priority face-id-node)
"
  Returns a NumberNode for the transition-priority of the face-id-node.
"
; Checks are not required b/c this function wouldn't be called
; without the context being satisfied
    (let* ((current-face-id (string->number (cog-name (gar
                (cog-execute!
                    (DefinedSchema "Current interaction target"))))))
           (face-id (string->number (cog-name face-id-node))))
       ; This is for when the room is empty. When room is empty the
        (if (equal? 0.0 current-face-id)
            (set-transition-priority! face-id ordinary-face-priority)
            (set-transition-priority! face-id
                (transition-priority face-id current-face-id))
        )
    )
)

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

    (define interaction-face
        (gar (cog-execute! (DefinedSchema "Current interaction target"))))

    ; Faces not interacted with
    (define faces
        (remove (lambda (x) (equal? interaction-face x)) (show-acked-faces)))

    ; If their is only one face being interacted with then continue
    ; interacting.
    (if (null? faces)
        interaction-face
        (let* ((face-ids
                    (par-map (lambda (x) (string->number (cog-name x))) faces))
               (wp (par-map (lambda (x) (weighted-priority x)) face-ids))
               (max-wp (get-maximum wp))
               (max-faces
                    (remove (lambda (x) (equal? 0 x))
                        (par-map (lambda (w f) (if (equal? w max-wp) f 0))
                            wp faces))))
            (list-ref max-faces
                    (random (length max-faces) (random-state-from-platform)))
        )
    )
)

; psi-action for it is time to change the face being interacted with.
; Update transition-priority and select face to interact with.
(DefineLink
	(DefinedPredicate "Change interaction target by priority")
	(SequentialAnd
        (DefinedPredicate "Update face transition-priorities")
		(True (Put
			(StateLink request-eye-contact-state (VariableNode "$fid"))
			(DefinedSchema "Select face by priority")))

		; Diagnostic print
		(Evaluation (GroundedPredicate "scm: print-msg")
			(ListLink (Node "Requested new interaction")))
	))

; -----------------------------------------------------------------------------
; The psi-rule Context
(Define
    (DefinedPredicate "Has person being intracted with changed?")
    (SequentialOr
        (Not (Equal
            (GetLink
                (TypedVariable (Variable "value") (Type "NumberNode"))
                (State prev-interaction-state (Variable "value")))
            (DefinedSchema "Current interaction target")))
    ))

(Define
    (DefinedSchema "Set previous interaction value")
    (Lambda
        (TypedVariable (Variable "value") (Type "NumberNode"))
        (State prev-interaction-state (Variable "value"))
    ))
