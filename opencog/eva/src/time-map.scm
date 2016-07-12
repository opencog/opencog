(use-modules (opencog) (opencog atom-types) (opencog ato pointmem)); needed for mapsi
(use-modules (opencog python))
;;initialize octomap with 15hz, 10 second or 150 frames buffer ; 1 cm spatial resolution
(create-map "faces" 0.01 66 150) (step-time-unit "faces")(auto-step-time-on "faces")
;;scm code
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
