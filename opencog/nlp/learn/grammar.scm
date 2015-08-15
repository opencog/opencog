;Author: Rohit Shinde
;Date: 23rd July, 2015

;This procedure gets all the MSTConnectors for a given disjunct
(define (get-mst-connector disjunct)
	(define disjunct-out-set (cog-outgoing-set disjunct))
	(define mst-connectors (cog-outgoing-set (cadr disjunct-out-set)))
	mst-connectors)

;This function gets the decoded disjunct for a given disjunct.
(define (get-decoded-disjunct disjunct)
	(define (create-actual-decoded-disjunct word-sign-list)
		(define disjunct-outset (cog-outgoing-set disjunct))
		(cog-atom-incr (LgWordCset
					(WordNode (cog-name (car disjunct-outset)))
					(LgAnd
						(decoded-connectors word-sign-list))) 1))
	(define (decoded-connectors word-sign-list)
		(define (helper-function word-sign)
			(DecodedConnector
				(WordNode (car word-sign))
				(LgConnDirNode (cdr word-sign))))
		(map helper-function word-sign-list))
	(define mst-connectors (get-mst-connector disjunct))
	(create-actual-decoded-disjunct (map get-connected-word mst-connectors)))

;This function gets the sign of the mst-connector
(define (get-sign-mst-connector mst-connector)
	(cog-name (cadr (cog-outgoing-set mst-connector))))

;This function gets all connected words of a given disjunct. The connected
;words are then passed to the (decoded-disjunct) function to get the 
;decoded disjunct of those words. The distance from the words is then
;calculated.
(define (get-disjunct-connected-words disjunct)
	(define mst-connectors (get-mst-connector disjunct))
	(define words-connected-to-disjunct (map get-connected-word mst-connectors))
	words-connected-to-disjunct)

;This function gets all the disjuncts for a given word.
(define (get-disjunct-from-word wordnode)
	(define inset (cog-incoming-set wordnode))
	(define (disjunct-filter? x)
		(cog-link? (cadr (cog-outgoing-set x))))
	(filter disjunct-filter? inset))

;This function creates decoded disjuncts for all the disjuncts of a given word
(define (create-decoded-disjuncts-for-all-disjuncts wordnode)
	(define all-disjuncts-of-word (get-disjunct-from-word wordnode))
	(map get-decoded-disjunct all-disjuncts-of-word))

;This function will create decoded disjuncts for all words connected to a
;given disjunct
(define (create-decoded-disjuncts-for-all-connected-words disjunct)
	(define word-sign-pairs (get-disjunct-connected-words disjunct))
	(map (lambda (x) 
			(get-decoded-disjunct 
				(car (get-disjunct-from-word (WordNode (car x))))))
				word-sign-pairs))

;This function gets the other word of a word pair. The input to this function
;is an MST-connector which defines the name of the connector as well as the
;sign.
(define (get-connected-word mst-connector)
	(define sign (get-sign-mst-connector mst-connector))
	(define connector-name (cog-name (car (cog-outgoing-set mst-connector))))
	(define in-set (cog-incoming-set (car (cog-outgoing-set mst-connector))))
	(define (helper-function sign mst-inset)
		(define lg-and-inset (cog-incoming-set (car mst-inset)))
		(define disj (car lg-and-inset))
		(define disj-outset (cog-outgoing-set disj))
		(define word (cog-name (car disj-outset)))
		(cons word sign))
	(if (equal? "-" sign)
		(if (equal? "+" (get-sign-mst-connector (car in-set)))
			(helper-function "-" (cog-incoming-set (car in-set)))
			(helper-function "+" (cog-incoming-set (cadr in-set))))
		(if (equal? "+" sign)
			(if (equal? "-" (get-sign-mst-connector (cadr in-set)))
				(helper-function "+" (cog-incoming-set (cadr in-set)))
				(helper-function "-" (cog-incoming-set (car in-set))))
			'())))

;This function uses all the above functions to create decoded disjuncts for all
;the disjuncts of a given wordnode as well as all the words connected to it.
(define (create-em-all-decoded-disjuncts wordnode)
	(define decoded-disjunct-of-word (create-decoded-disjuncts-for-all-disjuncts wordnode))
	(define disjuncts-of-the-wordnode (get-disjunct-from-word wordnode))
	(map create-decoded-disjuncts-for-all-connected-words disjuncts-of-the-wordnode))

