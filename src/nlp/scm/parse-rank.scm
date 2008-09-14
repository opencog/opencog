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

; Loop over SentenceNode
(define (prt-sent h) (display h) #f)

(define (prt-stuff h) (display (cog-incoming-set h)) #f)

(cog-map-type prt-stuff  'SentenceNode)

; need to look for ParseLink in the incoming set

(define (filter-plink proc data) 
	(cond 
		((null? data) #f)
		((eq? (caar data) 'ParseLink) 
			(display "hola") ) 
		(else (display "boogaloo") )
	)
	(filter-plink proc (cdr data))
	#f)

(define (filter-plink proc data) (display data) #f)
(define (filter-plink proc data) (display (pair? (car data))) #f)

need cog-atom? cog-stv? and also, type, name, 


(define (prt-stuff h) (filter-plink #t (cog-incoming-set h)) #f)
