;
; time-map.scm
;
; Functions that localize the current person who is speaking --
; these assign a face-id to the "who spoke" state, based on the
; direction the sound is coming from, as compared to the position
; of the face.
;
; XXX FIXME -- some of the below should be handled as psi-rules,
; instead of being hard-coded, here.  That is, we are interested
; in the locations of sound events in general, not just for determing
; who is speaking.
;
; Copyright 2016 Hanson Robotics
;
(use-modules (srfi srfi-1) )
(use-modules (opencog) (opencog atom-types)
	(opencog eva-model) (opencog eva-behavior))

(use-modules (opencog ato pointmem)); needed for maps

(define last-speaker (Concept "last person who spoke"))
(define prev-speaker (Concept "previous person who spoke"))

(State last-speaker (Number 0))
(State prev-speaker (Number 0))

; --------------------------------------------------------------------
(define facemap (SpaceMapNode "faces"))

; Create an opencog space-time map, for storing locations of faces.
; This map will be udates every 66 milliseconds (about 15 Hz), and
; will buffer 150 frames (about 10 seconds), and work with a one
; centimeter resolution (the native coords of the map are meters).
(create-map facemap (ListLink (Number 0.01) (Number 66) (Number 150)))

; Run the map in a new thread. This will automatically create a new
; time-slice every 66 milliseconds.
; XXX FIXME Is it wise to start this, just because the guile module got
; loaded? or should we have a distinct "start running it now" function?
(auto-step-time-on facemap)

; time-span is the amount of time in milliseconds to be considered
; for locating a face. The time limit for spatial memory for faces :-).
; The value is dependent on the frequency of update of the map and
; the number of frames, and is set as half the size of the total buffer.
(define face-loc-time-span (NumberNode 8000)) ; (8000 milliseconds or 8 seconds)

; get-last-locs-ato returns a SetLink holding AtLocationLinks
; of the form below:
;
;    (AtLocationLink
;      (SpaceMapNode "faces")
;      (ListLink
;         (NumberNode "42.000000")
;         (ListLink
;            (NumberNode "2.005000")
;            (NumberNode "1.005000")
;            (NumberNode "0.005000"))))
;
; `id-node` is "(NumberNode 42)" (the face id)
;
(define (get-last-location map-name id-node elapse)
	(define at-loc
		(gar (get-last-locs-ato map-name id-node elapse)))
	(if (null? at-loc)
		(ListLink (Number 0) (Number 0) (Number 0))
		(gddr at-loc)
	)
)

; ---------------------------------------------------------------------
;; get-face-coords - get the 3D position of a face.
;;
;; `FACE-ID-NODE` should be a Node holding a face-id.
;;
;; Given a face-id, this will get the last-known 3D (x,y,z) location
;; for that face from the space server.
;;
(define-public (get-face-coords FACE-ID)
	(get-last-location facemap FACE-ID face-loc-time-span))

(DefineLink
	(DefinedPredicate "look-at-face")
	(LambdaLink
		(Variable "$face-id")
		(Evaluation
			(DefinedPredicate "Do look at point")
			; The below returns a ListLink of 3D coords for the face.
			(ExecutionOutputLink
				(GroundedSchema "scm: get-face-coords")
				(ListLink (Variable "$face-id")))
		)))

(DefineLink
	(DefinedPredicate "glance-at-face")
	(LambdaLink
		(Variable "$face-id")
		(Evaluation
			(DefinedPredicate "Do gaze at point")
			; The below returns a ListLink of 3D coords for the face.
			(ExecutionOutputLink
				(GroundedSchema "scm: get-face-coords")
				(ListLink (Variable "$face-id")))
		)))

; ---------------------------------------------------------------------
;; Below creates say atom for face if sound came from it
;; XXX FIXME huh? this needs documentation.
;; XXX FIXME elminiate the use of cog-execute! -- that is not how
;; this should be designed -- these need to be learnable; and
;; cog-execute prevents learning.
;; XX This also bypasses the regular decision-making process
;; in the behvaior tree. So this is wrong in a bunch iof different ways.
(define (who-said? sent)
	;;request eye contact

	;;Debug below
	;(display "###### WHO SAID: ")
	;(display (cog-name
	;	(GetLink (TypedVariable (Variable "$fid") (TypeNode "NumberNode"))
	;		(State last-speaker (Variable "$fid")))))

	(cog-execute!
	(Put
		(State request-eye-contact-state (VariableNode "$fid"))
		(Get (State last-speaker (Variable "$fid")))
	))
	;;generate info
	(cog-execute!
	(PutLink
		(AtTimeLink
			(TimeNode (number->string (current-time)))
			(EvaluationLink
				(PredicateNode "say_face")
					(ListLink
						(VariableNode "$fid")
						(SentenceNode sent)))
				(Concept "sound-perception"))
		(Get (State last-speaker (Variable "$fid")))
	))
)

;;math
(define (dot-prod ax ay az bx by bz) (+ (* ax bx) (* ay by)(* az bz)))
(define (magnitude ax ay az) (sqrt (+ (* ax ax) (* ay ay) (* az az))))
(define (angle ax ay az bx by bz)
	(let* ((dp (dot-prod ax ay az bx by bz))
			(denom (* (magnitude ax ay az)(magnitude bx by bz))))
		(if (> denom 0)
			(acos (/ dp denom))
			0.0
		)
	)
)

; FACE-ID should be an atom identifying a face (currently,
; a NumberNode)
;assuming common zero in coordinates
;assuming sound was saved with co-oridinate transform applied for camera
;angle in radians
;
;; XXX FIXME -- this kind of crazy angle computation should be
;; happenening in the space-time server, and not here.
;;
(define (angle_face_id_snd FACE-ID xx yy zz)
	;; Convert from at location link to (x y z) list
	(define (space-nodes at-loc-link)
		(cog-outgoing-set (cadr
			(cog-outgoing-set (cadr (cog-outgoing-set at-loc-link))))))

	(define (loc-link-x at-loc-link)
		(string->number (cog-name (car (space-nodes at-loc-link)))))
	(define (loc-link-y at-loc-link)
		(string->number (cog-name (cadr (space-nodes at-loc-link)))))
	(define (loc-link-z at-loc-link)
		(string->number (cog-name (caddr (space-nodes at-loc-link)))))

	;; Get the xyz coords, as a list, for `face-id-node`
	(let* ((loc-atom (get-face-coords FACE-ID)))
		(if (null? loc-atom)
			6.2831853 ; two-pi
			(angle
				(loc-link-x loc-atom)
				(loc-link-y loc-atom)
				(loc-link-z loc-atom)
				xx yy zz)
		)
	)
)


;; face-nearest-sound -- get the face-id atom nearest the sound direction.
;;
;; A face is near if the sound direction is within 15 degrees of
;; the face.  Returns the atom for the face, or the emtpy list.
;;
;; XXX FIXME -- this kind of tulity needs to be in the space-time
;; server, and not here.
(define (face-nearest-sound xx yy zz)

	; The visible faces are stored as EvaluationLinks, attached
	; to the predicate "visible face". This function returns a
	; list of nodes (NumberNodes, actually) holding the face id's.
	(define (get-visible-faces)
	   (define visible-face (PredicateNode "visible face"))
		(filter (lambda(y) (equal? (cog-type y) 'NumberNode))
			(map (lambda (x) (car (cog-outgoing-set x)))
				(cog-chase-link 'EvaluationLink 'ListLink visible-face))))

	; This converts the list of visible faces into a list of faces
	; followed by thier curent 3D xyz coordinates.
	(define face-list
		(map
			(lambda (ATOM)
				(list ATOM (angle_face_id_snd ATOM xx yy zz)))
			 (get-visible-faces)))

	; If there are several faces, pick the nearest.
	; This code is squirrly, there must be a more elegant,
	; easier-to-understand way of doing this.
	(if (< (length face-list) 1)
		(list) ; zero faces
		(let* ((alist (append-map (lambda (x)(cdr x)) face-list))
				(amin (fold (lambda (n p) (min (abs p) (abs n)))
					(car alist) alist)))
			; discard faces that are more than 15 degrees away
			(if (> (* 3.1415926 (/ 15.0 180.0)) amin)
				(car (car (filter
					(lambda (x) (> (+ amin 0.0001) (abs (cadr x)))) face-list)))
				(list)
			)
		)
	)
)

;; This needs to be define-public, so that ros-bridge can send this
;; to the cogserver.
;; XXX TODO -- this should eventually be a psi-rule, so that we can
;; associate spoken sounds with speakers, but also know the locations
;; of loud sounds.  That is, the time-server needs to get sound
;; direction, no matter what.
(define-public (map-sound xx yy zz)
	(define fid (face-nearest-sound xx yy zz))
	(if (not (null? fid)) (StateLink last-speaker fid))
)
