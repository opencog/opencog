(use-modules (srfi srfi-1) )
(use-modules (opencog) (opencog atom-types) (opencog eva-behavior))
(use-modules (opencog ato pointmem)); needed for mapsi
(use-modules (opencog python))
;;initialize octomap with 15hz, 10 second or 150 frames buffer ; 1 cm spatial resolution
(create-map "faces" 0.01 66 150) (step-time-unit "faces") (auto-step-time-on "faces")
(create-map "sounds" 0.01 66 150) (step-time-unit "sounds") (auto-step-time-on "sounds")

;(map-ato "faces" (NumberNode "1") 1 2 3)
;(map-ato "sounds" (NumberNode "1") 1 2 3)

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
;;these functions assume only one location for one atom in a map at a time
(define (get-last-xyz map-name id-node elapse)
		(let* ((loc-atom (get-last-locs-ato map-name id-node elapse) ))
			(if (equal? (cog-atom (cog-undefined-handle)) loc-atom) ""
				(let* ((loc-link (car (cog-outgoing-set loc-atom)))
					(xx (number->string (loc-link-x loc-link)))
					(yy (number->string (loc-link-y loc-link)))
					(zz (number->string (loc-link-z loc-link))))
						(string-append xx " " yy " " zz))
			)
		)
)
;;get first occurence in map
(define (get-first-xyz map-name id-node elapse)
		(let* ((loc-atom (get-first-locs-ato map-name id-node elapse) ))
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
 (get-last-xyz "faces" face-id-node 500)
)

(define (get-snd-loc snd-id-node)
 ;(get-last-xyz "sounds" snd-id-node 5000)
 (get-first-xyz "sounds" snd-id-node 1200)
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
;assuming common zero in coordinates
;assuming sound was saved with co-oridinate transform applied for camera
;angle in radians
(define (angle_face_snd face-id snd-id)
  (let* ((fc (get-face (NumberNode face-id))) (sn (get-snd-1)) )
	(if (equal? fc "") (* 2 3.142)
										(if (equal? sn "") (* 2 3.142)
												(angle_bw fc sn)
										)
	))
)
(define (angle_face_snd1 face-id)
	(angle_face_snd face-id (get-snd-1))
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

(define (show-visible-faces)
        (define visible-face (PredicateNode "visible face"))
        (map (lambda (x) (car (cog-outgoing-set x)))
        (cog-chase-link 'EvaluationLink 'ListLink visible-face)))

;;get string of face-id's seperated by space - call python split() function
;;in scheme itself list of number nodes
(define (get-visible-faces)
  (cog-filter 'NumberNode (show-visible-faces))
	;;(string-concatenate (map (lambda (x)(string-append (cog-name x) " ")) (cog-filter 'NumberNode (show-visible-faces)) ))
)
;(define (get-visible-faces)(list (NumberNode "1")(NumberNode "2")))
;;below returns face id of face nearest to sound vector atleast 10 degrees, or 0 face id
(define (snd1-nearest-face)
	(let* ((lst (get-visible-faces))
				 (falist (map (lambda (x)(list (string->number (cog-name x)) (angle_face_snd1 (cog-name x)))) lst)))
		(if (< (length falist) 1)
			0
			(let* ((alist (append-map (lambda (x)(cdr x))falist))
						(amin (fold (lambda (n p) (min (abs p) (abs n))) (car alist) alist)))
			(if (> (/ (* 3.142 10.0) 180.0) amin)
				(car (car (filter (lambda (x) (> (+ amin 0.0001) (abs (cadr x)))) falist)))
				0
			))
		)
	)
)

;;below creates say atom for face if sound came from it
(define (who-said? sent)
	(let* ((fid (snd1-nearest-face)))
		(if (> fid 0)
			(
			;;request eye contact
			(StateLink request-eye-contact-state (NumberNode fid))
			;;generate info
			(AtTimeLink
				(TimeNode (number->string (current-time)))
				(EvaluationLink
					(PredicateNode "say")
					(ListLink (ConceptNode (number->string fid))(SentenceNode sent)))
					(ConceptNode "sound-perception"))
			)
		)
	)
)
