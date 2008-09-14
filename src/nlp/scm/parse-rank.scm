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

(define (do-stuff h) (display "zoom\n") #f)

; Look for ParseLink's in the incoming set
(define (process-parses h) 
	(cog-filter 'ParseLink do-stuff (cog-incoming-set h))
	#f
)

; Loop over all atoms of type SentenceNode, processing 
; each corresponding parse.
(cog-map-type process-parses 'SentenceNode)


