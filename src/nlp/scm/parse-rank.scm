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

; misc debugging crap
(define (prt-stuff h) (display h) #f)
(define (prt-inc h) (display (cog-incoming-set h)) #t)
(define (prt-word h) (display (cog-name h)) (newline) #f)

;
; provde a score for the parse
(define (score-parse h)

	; Accumulate words into a list; called from a deep nested loop.
	(define word-list '())
	(define (add-to-word-list h) (set! word-list (cons (cog-name h) word-list))  #f)

	; Make a list of pairs out of 'word' and 'word-list'
	; Return this list of pairs.
	(define (mk-pair word wd-list prs) 
		(if (eq? wd-list '())
			prs
			(mk-pair word (cdr wd-list) 
				(cons (cons word (car wd-list)) prs)
			)
		)
	)

	; Make a list of left-right pairs of all words in 'wlist'
	; That is, the right word of each pair must be to the right of the
	; left word in each pair (in sentential order). So, for example,
	; "Hello world" will produce only one pair: (hello . world) and
	; "Good moring world" produces three pairs: (good . morning)
	; (good . world) (morning . world)
	(define (mk-prs wlist prl)
		(if (eq? wlist '())
			prl
			(mk-prs (cdr wlist)
				(mk-pair (car wlist) (cdr wlist) prl))
		)
	)

	; Given a word-pair, look it up in the frequency/mutal-info database,
	; and associate a mutal info score to it. Return the scored pair, 
	; else return empty list.  So, example, the pair (pile .of) produces
	; ((pile . of) . 3.73598462236839) 
	(define (score-pair pr)
		(define qstr 
			(string-append
				"SELECT * FROM pairs WHERE left_word='"
				(car pr) "' AND right_word='" (cdr pr) "'"))

		(define row #f)
		; (display qstr) (newline)
		(dbi-query db-connection qstr)
		(set! row (dbi-get_row db-connection))
		(if (eq? row #f)
			'()
			(cons pr (cdr (assoc "mutual_info" row)))
		)
	)

	; Iterate over a list of pairs, and fetch word-pair scores from 
	; the SQL database. Returns an association list of scored pairs.
	(define (score-pairs pair-list slist)
		(if (eq? pair-list '())
			slist
			(let ((score (score-pair (car pair-list))))
				(if (eq? score '())
					(score-pairs (cdr pair-list) slist)
					(score-pairs (cdr pair-list) (cons score slist))
				)
			)
		)
	)

	; Iterate over a list of scored pairs, and return the highest-scoring
	; pair in the list.

	; Create a list of all of the word instances in the parse.
	(map-word-instances (lambda (z) (map-word-node add-to-word-list z)) h)

	; (display word-list)
	; (display (mk-prs word-list '()) )
	(display (score-pairs (mk-prs word-list '()) '()) )
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


