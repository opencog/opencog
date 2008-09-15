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

(use-modules (dbi dbi))
(define db-connection (dbi-open "postgresql" db-login))

(define (prt-stuff h) (display h) #f)
(define (prt-inc h) (display (cog-incoming-set h)) #t)

(define (prt-word h) (display (cog-name h)) (newline) #f)

(define (score-parse h)

	; Accumulate words into a list; called from a deep nested loop.
	(define word-list '())
	(define (add-to-word-list h) (set! word-list (cons (cog-name h) word-list))  #f)

	; A function that makes a list of pairs out of 'word' and 'word-list'
	(define (mk-pair word wd-list prs) 
		(if (eq? wd-list '())
			prs
			(mk-pair word (cdr wd-list) 
				(cons (cons word (car wd-list)) prs)
			)
		)
	)

	; A function that makes a list of pairs of all words in 'alist'
	; on the right, and 'blist' on the left.
	(define (mk-prs wlist prl)
		(if (eq? wlist '())
			prl
			(mk-prs (cdr wlist)
				(mk-pair (car wlist) (cdr wlist) prl))
		)
	)

	; Given a word-pair, look it up in the datapase, and assign
	; a mutal info score to it
	(define (score-pair pr)
		(cons 0.5 pr)
	)

	; a function that iterates over a list of pairs, and fetches
	; word-pair scores from the SQL database
	(define (score-pairs pair-list slist)
		(if (eq? pair-list '())
			slist
			(score-pairs (cdr pair-list)
				(cons (score-pair (car pair-list)) slist)
			)
		)
	)


	; Create a list of all of the word instances in the parse.
	(map-word-instances (lambda (z) (map-word-node add-to-word-list z)) h)

	; (display word-list)
	(display (mk-prs word-list '()))
	(newline)
	#f
)

; =============================================================

; Loop over all atoms of type SentenceNode, processing 
; each corresponding parse.
(cog-map-type 
	(lambda (x) (map-parses score-parse x))
	'SentenceNode
)


