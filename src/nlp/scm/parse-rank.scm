;
; parse-rank.scm
;
; Provide a score for a link-grammar parse, based on how well it matches
; a minimum spanning tree formed from word-pair mutual information.
;
; Requires access to a database containing previously computed word-pair
; mutual information values.
; 
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
; login information and database
(define db-login "linas:asdf:lexat:tcp:localhost:5432")

; (use-modules (dbi dbi))
; (define db-connection (dbi-open "postgresql" db-login))

(define (prt-stuff h) (display h) #t)
(define (prt-inc h) (display (cog-incoming-set h)) #t)

; =============================================================

; Loop over all atoms of type SentenceNode, processing 
; each corresponding parse.
(cog-map-type 
	(lambda (x) (map-parses 
		(lambda (y) (map-word-instances 
			(lambda (z) (map-word-node prt-stuff z))
			y)
		)
	 	x)
	)
	'SentenceNode
)


