;This is the MST module for creating MSTLinkNodes.
(use-modules (srfi srfi-1))

;Defines letters of the alphabet to be used in constructing a name for the 
;node.
(define letters "ABCDEFGHIJKLMNOPQRSTUVWXYZ")

;The function which actually constructs a name for the node.
(define (number->letters num)
  (unfold-right negative?
                (lambda (i) (string-ref letters (remainder i 26)))
                (lambda (i) (- (quotient i 26) 1))
                num))

;This function calls number->letters function and prepends an 'M' to the 
;generated name.
(define (number->tag num)
  (list->string (cons #\M (number->letters num))))

;This is a counter which will increment by 1 automatically on every usage.
(define (make-counter . x)
   ; validate argument
   (let ((count (if (and
                     (not (null? x))
                     (integer? (car x)))
                    (car x)
                    1)))
   ; return counter closure  
     (lambda ()
       (let ((current-count count))
         (set! count (+ 1 count))
         current-count))))

;make-counter is defined as counter to make its purpose clear.
(define counter (make-counter))

;The parse of any given sentence.
(define (parse text) (mst-parse-text text))

;Get the mutual information of the word pair.
(define (get-mutual-information wp) (car wp))

;The word pair has mutual information also embedded in it. This function strips
;that away and keeps only the two wordnodes.
(define (actual-wp wp) (car (cdr wp)))

;Get the first wordnode in the wordpair.
(define (first-wordnode wp) (car (actual-wp wp)))

;Get the second wordnode in the wordpair.
(define (second-wordnode wp) (car (cdr (actual-wp wp))))

;The wordnodes also have some superfluous information. This function removes
;that for the first wordnode.
(define (get-first-word wp) (car (cdr (first-wordnode wp))))

;The wordnodes also have some superfluous information. This function removes
;that for the second wordnode.
(define (get-second-word wp) (car (cdr (second-wordnode wp))))

;This function returns the atom which has the MSTLinkNode from the given set
;of incoming atoms for a given atom. In short, this function checks whether 
;the EvaluationLink already exists or is there a need to create a new 
;EvaluationLink. If the EvaluationLink exists, then that atoms is returned
;and the calling function will increment its CTV.
(define (get-mst-node in)
	(define temp (car in))
	(define outgoingset (cog-outgoing-set (car in)))
	(define node (car outgoingset))
	(if (eq? 'MSTLinkNode (cog-type node))
		temp
		(if (pair? (cdr in))
			(get-mst-node (cdr in))
			'())))

;This function creates an EvaluationLink for each word pair which has not been
;seen previously.
(define (create-evaluation-link wp)
	(define x (get-first-word wp))
	(define y (get-second-word wp))
	(define ll (cog-link 'ListLink x y))
	(define incoming (cog-incoming-set ll))
	(define mstnode (get-mst-node incoming))
	(if (null? mstnode)
		(EvaluationLink (MSTLinkNode (number->tag (counter))) (ListLink x y))
		(cog-atom-incr mstnode 1)))

;This function is used by the higher-order-function filter to selectively apply
;the create-evaluation-link function to only those word pairs which do not 
;have a mutual information of -1000.
(define (criteria? wp)
	(if (= -1000 (get-mutual-information wp))
		#f
		#t))

<<<<<<< HEAD
=======
;Applies the map function to create MST nodes for all the word pairs. It 
;creates nodes for only those word pairs which have a mutual information not 
;equal to -1000
(define (create-MST-nodes text)
	(map create-evaluation-link (filter criteria? (parse text)) ))

;This function will give the name of the MSTNode which exists in a given 
;Evaluation Link.
(define (get-MST-name outset)
	(cog-name (car outset)))

;This function will return the first word present in the ListLink component of 
;an EvaluationLink.
(define (get-first-evaluation-word outset)
	(cog-name (car (cog-outgoing-set (car (cdr outset))))))

;This function will return the second word present in the ListLink component of  
;an EvaluationLink.
(define (get-second-evaluation-word outset)
	(cog-name (car (cdr (cog-outgoing-set (car (cdr outset)))))))

;This function will get all the unique words that were present in the sentence
;for which the MST nodes were created.
(define (get-sentence-words sentence)
	(string-split sentence #\ ))

;The extract-disjunct function returns a list of disjuncts for that particular 
;word. However, that list contains many null lists also. These lists need to 
;be eliminated before they can be processed further.
(define (filter-disjunct disjuncts)
	(filter filter-disjunct-criteria? disjuncts))

;This is the criteria for the filter function which cleans up the disjuncts 
;list for a list which does not contain any null lists.
(define (filter-disjunct-criteria? x)
	(not (null? x)))

;This function returns the connector of the disjunct. Eg:- "MC+" will return MC
(define (get-disjunct-connector disjunct)
	(substring disjunct 0 (- (string-length disjunct) 1)))

;This function returns the sign of the disjunct. Eg:- "MC+" will return +
(define (get-disjunct-sign disjunct)
	(substring disjunct (- (string-length disjunct) 1)))

;This function takes as input a word and a list of MST nodes created by the 
;create-MST-nodes function. For the given word, it gives the disjuncts of that 
;word as found in the list of nodes give to it as input.
(define (extract-disjunct word nodes)
	(if (not (null? nodes))
		(let* (
				[node (car nodes)]
				[outset (cog-outgoing-set node)]
				[linkname (get-MST-name outset)]
				[firstword (get-first-evaluation-word outset)]
				[secondword (get-second-evaluation-word outset)])
		(if (equal? word firstword)
			(if (null? nodes)
				'()
				(cons (string-append linkname "+") (extract-disjunct word (cdr nodes))))
			(if (equal? word secondword)
				(if (null? nodes)
					'()
					(cons (string-append linkname "-") (extract-disjunct word (cdr nodes))))
				(if (null? nodes)
					'()
					(cons '() (extract-disjunct word (cdr nodes)))))))
		'()))

;Given a list of disjuncts, it will create MSTConnector atoms and return a list
;of such atoms. This is useful because it is difficult otherwise to create 
;atoms for LgWordCset. 
(define (create-connector-atoms disjuncts)
	(if (not (null? disjuncts))
		(let* (
				[disjunct (car disjuncts)])
			(cons
				(MSTConnector
					(LgConnectorNode (get-disjunct-connector disjunct))
					(LgConnDirNode (get-disjunct-sign disjunct))) (create-connector-atoms (cdr disjuncts))))
		'()))

;This function will create the LgWordCset atom which will have a WordNode and 
;also the disjuncts for that given WordNode.
(define (create-disjunct word disjuncts)
	(if (not (null? (create-connector-atoms disjuncts)))
		(LgWordCset
			(WordNode word)
			(LgAnd
				(create-connector-atoms disjuncts)))
		'()))

;This function extracts the disjuncts for the given sentence for each and every
;word present in the sentence. It returns a list of disjuncts for a sentence. 
;Each disjunct in the disjunctslist is a list itself. It corresponds to a word
;in the input sentence.
(define (loop-over-words words nodes)
	(if (not (null? words))
		(cons (extract-disjunct (car words) nodes) (loop-over-words (cdr words) nodes))
		'()))

;This procedure is called by the make-disjuncts procedure. It is mainly written
;so that I can recursively call this procedure itself to make new LgWordCset
;atoms.
(define (pseudo-make-disjuncts words disjunctslist)
	(if (not (null? words))
		(let (
			[word (car words)]
			[disjuncts (car disjunctslist)])
			(create-disjunct word (filter-disjunct disjuncts))
			(pseudo-make-disjuncts (cdr words) (cdr disjunctslist)))
		))

;This is the final procedure which (directly or indirectly) uses all the
;procedures described above. It takes as input any text, and creates
;the LgWordCset atoms along with the appropriate MSTConnector atoms.
(define (make-disjuncts text)
	(define nodes (create-MST-nodes text))
	(define words (get-sentence-words text))
	(define disjunctslist (loop-over-words words nodes))
	(pseudo-make-disjuncts words disjunctslist))

