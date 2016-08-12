(use-modules (opencog))

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

; Priority scale used for faces.
(define lowest-face-priority  0.0)
(define ordinary-face-priority 0.5)
(define highest-face-priority 1.0)

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
        (VariableList (Variable "face-id") (Variable "priority"))
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
    (gdar (cog-execute!
        (PutLink
            (DefinedSchema "Get face priority")
            (Number face-id))))
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
            ; The GetLink is structuraly the same as
            ; (DefinedSchema "Get face priority")
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

; -----------------------------------------------------------------------------
