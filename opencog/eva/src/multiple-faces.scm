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
; usage:
;(cog-evaluate! (PutLink
;    (DefinedPredicate "Set face priority")
;    (List (Number "12") (Number ".12"))))
(DefineLink
	(DefinedPredicate "Set face priority")
    (Lambda
        (VariableList (Variable "face-id") (Variable "priority"))
        (True (StateLink
                (List
                    (Concept "visual priority")
                    (Variable "face-id"))
                (Variable "priority")))))

(define (set-priority face-id priority)
    (cog-evaluate! (PutLink
        (DefinedPredicate "Set face priority")
        (List (Number face-id) (Number priority))))
)

(define (get-priority face-id)
    (gar (cog-execute! (Get
        (StateLink
            (List
                (Concept "visual priority")
                (Number face-id))
            (Variable "priority")))))
)
