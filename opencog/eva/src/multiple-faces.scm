(use-modules (opencog))

(load "time-map.scm")
(load "self-model.scm")

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
(define (get-face-coordinate face-id)
"
  Returns a list (x y z)
"
    (let ((coord (get-last-xyz "faces" (Number face-id) face-loc-time-span)))
        (if (equal? "" coord)
            '()
            (map string->number (string-split coord #\ ))
        )
    )
)

(define (get-face-coordinate-in-plane-yz face-id)
    (let ((new-x distance)
          (xyz (get-face-coordinate face-id)))
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
        (if (or (null? coord-1) (null? coord-1))
            (inf)
            (sqrt (+
                (expt (- (list-ref coord-1 1) (list-ref coord-2 1)) 2.0)
                (expt (- (list-ref coord-1 2) (list-ref coord-2 2)) 2.0)))
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
