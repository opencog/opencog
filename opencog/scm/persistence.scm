;
; persistence.scm
;
; Utilities related to atom persistence.
; The libpersist.so module must be loaded, in order for these to work.
;
; Copyright (C) 2009 Linas Vepstas <linasvepstas@gmail.com>
;
; --------------------------------------------------------------------
(define (store-referers atomo)
"
 store-referers -- Store to SQL all hypergraphs that contain given atom

 This stores all hypergraphs that the given atom participates in.
 It does this by recursively exploring the incoming set of the atom.
"
	(define (do-store atom)
		(let ((iset (cog-incoming-set atom)))
			(if (null? iset)
				(store-atom atom)
				(for-each do-store iset)
			)
		)
	)
	(do-store atomo)
)

; --------------------------------------------------------------------
(define (load-referers atom)
"
 load-referers -- Load from SQL all hypergraphs that contain given atom 

 This loads all hypergraphs that the given atom participates in.
 It does this by recursively exploring the incoming set of the atom.
"
	(if (not (null? atom))
		; The fetch-incoming-set function for this is defined to perform
		; a recursive fetch.
		; We do an extra recursion here, in case we were passed a list.
		(if (pair? atom)
			(for-each load-referers atom)
			(fetch-incoming-set atom)
		)
	)
)

; --------------------------------------------------------------------
