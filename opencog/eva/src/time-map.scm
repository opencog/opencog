(use-modules (opencog) (opencog atom-types) (opencog eva-behavior))
(use-modules (opencog ato pointmem)); needed for mapsi
(use-modules (opencog python))
;;initialize octomap with 15hz, 10 second or 150 frames buffer ; 1 cm spatial resolution
(create-map "faces" 0.01 66 150) (step-time-unit "faces")(auto-step-time-on "faces")
(create-map "sounds" 0.01 100 100) (step-time-unit "sounds")(auto-step-time-on "sounds")

;;returns null string if atom not found, number x y z string if okay
;;these functions assume only one location for one atom in a map at a time
(define (get-past-xyz map-name id-node elapse)
		(let* ((loc-atom (get-past-locs-ato map-name id-node elapse) ))
			(if (equal? (cog-atom (cog-undefined-handle)) loc-atom) ""
				(let* ((loc-link (car (cog-outgoing-set loc-atom)))
					(xx (number->string (loc-link-x loc-link)))
					(yy (number->string (loc-link-y loc-link)))
					(zz (number->string (loc-link-z loc-link))))
						(string-append xx " " yy " " zz))
			)
		)
)
;;returns null string if atom not found, number x y z string if okay
(define (get-xyz map-name id-node)
		(let* ((loc-atom (get-locs-ato map-name id-node) ))
			(if (equal? (cog-atom (cog-undefined-handle)) loc-atom) ""
				(let* ((loc-link (car (cog-outgoing-set loc-atom)))
					(xx (number->string (loc-link-x loc-link)))
					(yy (number->string (loc-link-y loc-link)))
					(zz (number->string (loc-link-z loc-link))))
						(string-append xx " " yy " " zz))
			)
		)
)

;;scm code
(define (get-face face-id-node)
 (get-past-xyz "faces" face-id-node 1)
)

(define (get-snd-loc snd-id-node)
 (get-past-xyz "sounds" snd-id-node 1)
)
;;sound id 1
(define (save-snd-1 x y z)
	(map-ato "sounds" (NumberNode "1") x y z)
)
(define (get-snd-1)
	(get-snd-loc (NumberNode "1"))
)

;;math
(define (dot-prod ax ay az bx by bz) (+ (* ax bx) (* ay by)(* az bz)))
(define (magnitude ax ay az) (sqrt (+ (* ax ax) (* ay ay) (* az az))))
(define (angle ax ay az bx by bz)
	(let* ((dp (dot-prod ax ay az bx by bz))(denom (* (magnitude ax ay az)(magnitude bx by bz))))
	(if (> denom 0) (acos (/ dp denom)) 0.0))
)

(define (angle_bw xyz-a xyz-b)
	(let* ((sta (string-split xyz-a #\ ))
		(stb (string-split xyz-b #\ ))
		(ax (string->number(car sta)))
		(ay (string->number(cadr sta)))
		(az (string->number(caddr sta)))
		(bx (string->number(car stb)))
		(by (string->number(cadr stb)))
		(bz (string->number(caddr stb))))
		(angle ax ay az bx by bz))
)

;assuming sound was saved with co-oridinate transform applied for camera
;angle in radians
(define (angle_face_snd face-id snd-id)
	(angle (get-face (NumberNode face-id-node)) (get-snd-loc (NumberNode snd-id-node)))
)
(define (angle_face_snd1 face-id)
	(angle (get-face (NumberNode face-id-node)) (get-snd-1))
)
;;get all face-ids and only one sound id 1.0, compare them

(define (look-at-face face-id-node)
		(let* ((loc-atom (get-past-locs-ato "faces" face-id-node 1))
			(fnc "look_at_face_point("))
			(if (equal? (cog-atom (cog-undefined-handle)) loc-atom) (stv 0 0)
				(let* ((loc-link (car (cog-outgoing-set loc-atom)))
					(xx (number->string (loc-link-x loc-link)))
					(yy (number->string (loc-link-y loc-link)))
					(zz (number->string (loc-link-z loc-link))))
						(python-eval (string-append fnc xx "," yy "," zz ")")))

			)
		)
		(stv 1 1)
)
