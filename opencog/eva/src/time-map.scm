(use-modules (srfi srfi-1) )
(use-modules (opencog) (opencog atom-types) (opencog eva-model) (opencog eva-behavior))
(use-modules (opencog ato pointmem)); needed for maps
(use-modules (opencog python))

(StateLink (ConceptNode "last person who spoke") (NumberNode "0"))
; -----------------------------------------------------------------------------
; For recording facial coordinates, create octomap with 15hz, 10 second or
; 150 frames buffer and 1 cm spatial resolution.
(create-map "faces" 0.01 66 150)
; Initialize  the map
(step-time-unit "faces")
; Make the stepping take place automatically
(auto-step-time-on "faces")
; time-span is the amount of time in milliseconds to be considered for locating
; a face. The time limit for spatial memory for faces :-). The value is
; dependent on the frequency of update of the map and the number of frames, and
; is set as half the size of the total buffer.
(define face-loc-time-span 8000) ; (8000 milliseconds or 8 seconds)

; -----------------------------------------------------------------------------
; For recording sound coordinates, create octomap with 10hz, 10 second or
; 100 frames buffer and 0.1 cm spatial resolution.
(create-map "sounds" 0.001 50 200)
; Initialize  the map
(step-time-unit "sounds")
; Make the stepping take place automatically
(auto-step-time-on "sounds")

; -----------------------------------------------------------------------------

;;returns null string if atom not found, number x y z string if okay
;;these functions assume only one location for one atom in a map at a time
(define (get-last-xyz map-name id-node elapse)
	(let* ((loc-atom (get-last-locs-ato map-name id-node elapse) ))
		(if (equal? (cog-atom (cog-undefined-handle)) loc-atom)
			(list )
			(let* ((loc-link (car (cog-outgoing-set loc-atom)))
				(xx (loc-link-x loc-link))
				(yy (loc-link-y loc-link))
				(zz (loc-link-z loc-link)))
					(list xx yy zz))
		)
	)
)
;;get first occurence in map
(define (get-first-xyz map-name id-node elapse)
		(let* ((loc-atom (get-first-locs-ato map-name id-node elapse) ))
			(if (equal? (cog-atom (cog-undefined-handle)) loc-atom)
				(list )
				(let* ((loc-link (car (cog-outgoing-set loc-atom)))
					(xx (loc-link-x loc-link))
					(yy (loc-link-y loc-link))
					(zz (loc-link-z loc-link)))
						(list xx yy zz))
			)
		)
)

;;scm code
(define (get-face face-id-node e-start)
 (get-last-xyz "faces" face-id-node (round e-start))
)

;;sound id 1
(define (save-snd-1 x y z)
	(map-ato "sounds" (NumberNode "1") x y z)
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

;assuming common zero in coordinates
;assuming sound was saved with co-oridinate transform applied for camera
;angle in radians

(define (angle_face_id_snd face-id xx yy zz)
	(let* ((fc (get-face (NumberNode face-id) 600)))
		(if (null? fc)
			(* 2 3.142)
			(angle (car fc) (cadr fc) (caddr fc) xx yy zz)
		)
	)
)

;;;searches for last face id location observed in previous 8 seconds
;;if face id is present in past 1 and 2 seconds then past 1 second id will be recovered
;;then python look at point function is called which calls blender api to look at face
;;look at face also sets gaze at face
;; expects face-id number node
(define (look-at-face face-id-node)
	(let ((loc-atom
			(get-last-locs-ato "faces" face-id-node face-loc-time-span)))
		(if (equal? (cog-atom (cog-undefined-handle)) loc-atom)
			(stv 1 1) ; FIXME: How should it be handled when (stv 0 1) is returned
			(let* ((loc-link (car (cog-outgoing-set loc-atom)))
					(xx (number->string (loc-link-x loc-link)))
					(yy (number->string (loc-link-y loc-link)))
					(zz (number->string (loc-link-z loc-link))))
				(python-eval
					(string-append "look_at_face_point(" xx "," yy "," zz ")"))
				(stv 1 1)
			)
		)
	)
)

;;;searches for last face id location observed in previous 8 seconds
;;if face id is present in past 1 and 2 seconds then past 1 second id will be recovered
;;then python gaze at point function is called which calls blender api to gaze at face
;;gaze at face only moves the eyes
;; expects face-id number node
(define (gaze-at-face face-id-node)
	(let ((loc-atom
			(get-last-locs-ato "faces" face-id-node face-loc-time-span)))
		(if (equal? (cog-atom (cog-undefined-handle)) loc-atom)
			(stv 1 1) ; FIXME: How should it be handled when (stv 0 1) is returned
			(let* ((loc-link (car (cog-outgoing-set loc-atom)))
					(xx (number->string (loc-link-x loc-link)))
					(yy (number->string (loc-link-y loc-link)))
					(zz (number->string (loc-link-z loc-link))))
				(python-eval
					(string-append "gaze_at_face_point(" xx "," yy "," zz ")"))
				(stv 1 1)
			)
		)
	)
)
;;get string of face-id's seperated by space - call python split() function
;;in scheme itself list of number nodes
(define (get-visible-faces)
  (cog-filter 'NumberNode (show-visible-faces))
)

;;below creates say atom for face if sound came from it
;;;;
(define (who-said? sent)
	;;request eye contact

	;;Debug below
	;(display "###### WHO SAID: ")
	;(display (cog-name
	;	(GetLink (TypedVariable (Variable "$fid") (TypeNode "NumberNode"))
	;		(StateLink
	;			(ConceptNode "last person who spoke")(VariableNode "$fid")))))

	(cog-execute!
	(PutLink
		(StateLink request-eye-contact-state (VariableNode "$fid"))
		(GetLink (TypedVariable (Variable "$fid") (TypeNode "NumberNode"))
			(StateLink
				(ConceptNode "last person who spoke") (VariableNode "$fid")))
	))
	;;generate info
	(cog-execute!
	(PutLink
	(AtTimeLink
		(TimeNode (number->string (current-time)))
		(EvaluationLink
			(PredicateNode "say_face")
				(ListLink
					(ConceptNode (cog-name (VariableNode "$fid")))
					(SentenceNode sent)))
			(ConceptNode "sound-perception"))
	(GetLink (TypedVariable (Variable "$fid") (TypeNode "NumberNode"))
		(StateLink
			(ConceptNode "last person who spoke") (VariableNode "$fid")))
	))
)

;;get all face-ids and only one sound id 1.0, compare them
;;threshold = sound in +-15 degrees of face
;below returns face id of face nearest to sound vector atleast 10 degrees, or
;0 face id
(define (snd-nearest-face xx yy zz)
	(let* ((lst (get-visible-faces))
			(falist (map
				(lambda (x) (list
					(string->number (cog-name x))
					(angle_face_id_snd (cog-name x) xx yy zz)))
				lst)))
		(if (< (length falist) 1)
			0
			(let* ((alist (append-map (lambda (x)(cdr x)) falist))
					(amin (fold (lambda (n p) (min (abs p) (abs n)))
						(car alist) alist)))
				(if (> (/ (* 3.142 15.0) 180.0) amin)
					(car (car (filter
						(lambda (x) (> (+ amin 0.0001) (abs (cadr x)))) falist)))
					0
				)
			)
		)
	)
)


(define (map-sound xx yy zz)
	(save-snd-1 xx yy zz)
	(let* ((fid (snd-nearest-face xx yy zz)))
		(if (> fid 0)
			(begin
			;;request eye contact
			;;(StateLink request-eye-contact-state (NumberNode fid))
			;;generate info
			(StateLink (ConceptNode "last person who spoke") (NumberNode fid))
			)
		)
	)
)
