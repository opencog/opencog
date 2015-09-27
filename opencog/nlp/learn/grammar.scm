;
; grammar.scm
;
; Copyright (c) 2015 Rohit Shinde
;

; ---------------------------------------------------------------------
; Get all the MSTConnectors for a given disjunct.
; For example, given an input of
;(LgWordCset (ctv 1 0 20)
;   	(WordNode "This" (ctv 1 0 470256))
;   	(LgAnd
;      		(MSTConnector
;       	  	(LgConnectorNode "MB")
;         		(LgConnDirNode "+")
;      		)
;   	)
;	)
; This will generate an output of 
;((MSTConnector
;   (LgConnectorNode "MB")
;   (LgConnDirNode "+")
;))
(define (get-mst-connector disjunct)
	(define disjunct-out-set (cog-outgoing-set disjunct))
	(define mst-connectors (cog-outgoing-set (cadr disjunct-out-set)))
	mst-connectors)

; ---------------------------------------------------------------------
; Get the decoded disjunct for a given MST disjunct.
; The 'decoded disjunct' is a disjunct, but with the abstract
; connector types replaced by the name of the word-classes that they
; connect to.  This is only possible when a word or word-class
; has been given a distinct name, and when connector types have not
; yet been clustered.  When this holds, a given connector type will
; connect uniquely to just one word-class; thus the word-class can
; be used as a synonym for the connector.  Note, however, the converse
; is not true: a word-class does not, cannot uniquely identify a
; connector.
;
; In the simplest example, before any clustering is performed, then all
; connectors are in unique, one-to-one correspondance with word-pairs.
; Thus, given a word, and a connector, the remote word is uniquely
; identified, and cna be used as a "stand-in" for the connector.
;
; Decoded disjuncts are used during clustering. The 'decoded disjuncts'
; associated with a word form the basis for a vector space. The
; similarity of different words can be modllled by the cosine distance
; between the vectors, and used as a metric for clustering.
;
; For example, given an input of 
; (LgWordCset (ctv 1 0 20)
;   (WordNode "This" (ctv 1 0 470256))
;   (LgAnd
;      (MSTConnector
;         (LgConnectorNode "MB")
;         (LgConnDirNode "+")
;      )
;   )
;)
;
; This will generate an output of 
;(LgWordCset (ctv 1 0 1)
;   (WordNode "This" (ctv 1 0 470256))
;    (LgAnd
;      (DecodedConnector
;         (WordNode "is" (ctv 1 0 5058187))
;         (LgConnDirNode "+")
;      )
;   )
;)

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

; ---------------------------------------------------------------------
; Get the sign of the mst-connector.
;
; For example, given an input of
;(MSTConnector
;   (LgConnectorNode "MB")
;   (LgConnDirNode "+")
;)
;
; This will generate an output of
; +
;
(define (get-sign-mst-connector mst-connector)
	(cog-name (cadr (cog-outgoing-set mst-connector))))

; ---------------------------------------------------------------------
; Gets all connected words of a given disjunct. The connected
; words are then passed to the (decoded-disjunct) function to get the
; decoded disjunct of those words. The distance from the words is then
; calculated.
;
; For example, given an input of
;(LgWordCset (ctv 1 0 20)
;   (WordNode "is" (ctv 1 0 5058187))
;   (LgAnd
;      (MSTConnector
;         (LgConnectorNode "MB")
;         (LgConnDirNode "-")
;      )
;      (MSTConnector
;         (LgConnectorNode "MC")
;         (LgConnDirNode "+")
;      )
;   )
;)
;
; The output is the following list:
;((This . -) (a . -))

(define (get-disjunct-connected-words disjunct)
	(define mst-connectors (get-mst-connector disjunct))
	(define words-connected-to-disjunct (map get-connected-word mst-connectors))
	words-connected-to-disjunct)

; ---------------------------------------------------------------------
; Gets all the disjuncts for a given word.
; For example, given an input of
; (WordNode "is")
; This will return
; 
;((LgWordCset (ctv 1 0 1)
;   (WordNode "is" (ctv 1 0 5058187))
;   (LgAnd
;      (MSTConnector
;         (LgConnectorNode "MB")
;         (LgConnDirNode "-")
;     )
;     (MSTConnector
;         (LgConnectorNode "MG")
;         (LgConnDirNode "+")
;      )
;      (MSTConnector
;         (LgConnectorNode "MH")
;         (LgConnDirNode "+")
;      )
;   )
;)
; (LgWordCset (ctv 1 0 20)
;   (WordNode "is" (ctv 1 0 5058187))
;   (LgAnd
;      (MSTConnector
;         (LgConnectorNode "MB")
;         (LgConnDirNode "-")
;      )
;      (MSTConnector
;         (LgConnectorNode "MC")
;         (LgConnDirNode "+")
;      )
;   )
;)
;)
(define (get-disjunct-from-word wordnode)
	(define inset (cog-incoming-set wordnode))
	(define (disjunct-filter? x)
		(cog-link? (cadr (cog-outgoing-set x))))
	(filter disjunct-filter? inset))

; ---------------------------------------------------------------------
; Creates decoded disjuncts for all the disjuncts of a given word
; For example, given an input of
; (WordNode "is")
; This will return
; ;((LgWordCset (ctv 1 0 1)
;   (WordNode "is" (ctv 1 0 5058187))
;   (LgAnd
;      (DecodedConnector
;         (WordNode "This" (ctv 1 0 470256))
;         (LgConnDirNode "-")
;      )
;      (DecodedConnector
;         (WordNode "another" (ctv 1 0 109754))
;         (LgConnDirNode "-")
;      )
;      (DecodedConnector
;         (WordNode "test" (ctv 1 0 22502))
;         (LgConnDirNode "+")
;      )
;   )
;)
; (LgWordCset (ctv 1 0 1)
;   (WordNode "is" (ctv 1 0 5058187))
;   (LgAnd
;      (DecodedConnector
;         (WordNode "This" (ctv 1 0 470256))
;         (LgConnDirNode "-")
;      )
;      (DecodedConnector
;         (WordNode "a" (ctv 1 0 5658964))
;         (LgConnDirNode "-")
;      )
;   )
;)
;)
;
(define (create-decoded-disjuncts-for-all-disjuncts wordnode)
	(define all-disjuncts-of-word (get-disjunct-from-word wordnode))
	(map get-decoded-disjunct all-disjuncts-of-word))

; ---------------------------------------------------------------------
; Create decoded disjuncts for all words connected to a
; given disjunct
; Given the following structure as input
;	(LgWordCset (ctv 1 0 20)
;   	(WordNode "This" (ctv 1 0 470256))
;   	(LgAnd
;      		(MSTConnector
;       	  	(LgConnectorNode "MB")
;         		(LgConnDirNode "+")
;      		)
;   	)
;	)
;The output of this function is:
; A list of all decoded disjuncts for all words connected to a particular disjunct
(define (create-decoded-disjuncts-for-all-connected-words disjunct)
	(define word-sign-pairs (get-disjunct-connected-words disjunct))
	(map (lambda (x)
			(get-decoded-disjunct
				(car (get-disjunct-from-word (WordNode (car x))))))
				word-sign-pairs))

; ---------------------------------------------------------------------
; Get the other word of a word pair. The input to this function
; is an MST-connector which defines the name of the connector as well
; as the sign.
;The input to this function is the following structure:
;(MSTConnector
;   (LgConnectorNode "MB")
;   (LgConnDirNode "+")
;)
;The output is the following:
;(WordNode "XYZ")
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

; ---------------------------------------------------------------------
; This function uses all the above functions to create decoded disjuncts
; for all the disjuncts of a given wordnode as well as all the words
; connected to it.
(define (create-em-all-decoded-disjuncts wordnode)
	(define decoded-disjunct-of-word (create-decoded-disjuncts-for-all-disjuncts wordnode))
	(define disjuncts-of-the-wordnode (get-disjunct-from-word wordnode))
	(map create-decoded-disjuncts-for-all-connected-words disjuncts-of-the-wordnode))

