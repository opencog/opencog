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
(use-modules (opencog python))

(define last-speaker (ConceptNode "last person who spoke"))
(define prev-speaker (ConceptNode "previous person who spoke"))

(StateLink last-speaker (NumberNode "0"))
(StateLink prev-speaker (NumberNode "0"))

; --------------------------------------------------------------------
; Create an opencog space-time map, for storing locations of faces.
; This map will be udates every 66 millisdeconds (about 15 Hz), and
; will buffer 150 frames (about 10 seconds), and work with a one
; centimeter resolution (the native coords of the map are meters).
(create-map "faces" 0.01 66 150)

; Run the map in a new thread. This will autoamtically create a new
; time-slice every 66 milliseconds.
; XXX FIXME Is it wise to start this, just because the guile module got
; loaded? or should we have a distinct "start running it now" function?
(auto-step-time-on "faces")

; time-span is the amount of time in milliseconds to be considered
; for locating a face. The time limit for spatial memory for faces :-).
; The value is dependent on the frequency of update of the map and
; the number of frames, and is set as half the size of the total buffer.
(define face-loc-time-span 8000) ; (8000 milliseconds or 8 seconds)

; ---------------------------------------------------------------------
;; Given the atom `id-node`, this fetches the last known location
;; (3D xyz coordinates) for that atom in the map `map-name`.
;; The search will be made only in the most recent time interval of
;; `elapse` millisconds.
;;
;; Returns a list of three floating-point numbers, or a null list
;; if the atom is not found.
;;
;; This function assumes that an atom can have only one location at a
;; time.
(define-public (get-last-xyz map-name id-node elapse)
	;
	; get-last-locs-ato returns a SetLink holding AtLocationLinks
	; of the form below:
	;
	;    (AtLocationLink
	;      (ObjectNode "faces")
	;      (ListLink
	;         (NumberNode "42.000000" (av 5 0 0))
	;         (ListLink
	;            (NumberNode "2.005000")
	;            (NumberNode "1.005000")
	;            (NumberNode "0.005000"))))
	;
	; Here, `map-name` is "faces"
	; `id-node` is "(NumberNode 42)" (the face id)
	;
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


; ---------------------------------------------------------------------
;; get-face-coords - get the 3D position of a face.
;;
;; `FACE-ID-NODE` should be a Node holding a face-id.
;;
;; Given a face-id, this will get the last-known 3D (x,y,z) location
;; for that face from the space server.
;;
;; XXX FIXME - this needs to be a part of the space-time server.
;; That is, the space-time server should provide a native interface
;; for converting object-id's into 3D coordinates!  We should not
;; have to write hacky scheme code to accomplish this.
;;
(define-public (get-face-coords FACE-ID)
	;; Get the xyz coords, as a list, for `face-id-node`
	(define (get-face face-id-node e-start)
		(get-last-xyz "faces" face-id-node (round e-start))
	)
	; Get the x,y,z coords.
	(define xyz-list (get-face FACE-ID face-loc-time-span))
	(define x "0.0")
	(define y "0.0")
	(define z "0.0")
	; The list might be empty, in which case ther is no face.
	(if (not (null? xyz-list))
		(begin
			(set! x (number->string (car xyz-list)))
			(set! y (number->string (cadr xyz-list)))
			(set! z (number->string (caddr xyz-list)))
		))

	;; XXX FIXME we should throw, here, if no such face-id
	(ListLink
		(NumberNode x)
		(NumberNode y)
		(NumberNode z))
)

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
(define (angle_face_id_snd FACE-ID xx yy zz)
	;; Get the xyz coords, as a list, for `face-id-node`
	(define (get-face face-id-node e-start)
		(get-last-xyz "faces" face-id-node (round e-start)))
	(let* ((fc (get-face FACE-ID face-loc-time-span)))
		(if (null? fc)
			6.2831853 ; two-pi
			(angle (car fc) (cadr fc) (caddr fc) xx yy zz)
		)
	)
)


;; face-nearest-sound -- get the face-id atom nearest the sound direction.
;;
;; A face is near if the sound direction is within 15 degrees of
;; the face.  Returns the atom for the face, or the emtpy list.
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
