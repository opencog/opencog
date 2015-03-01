;This is the MST module for creating MSTLinkNodes.
(use-modules (srfi srfi-1))

;Defines letters of the alphabet to be used in constructing a name for the node.
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

;Applies the map function to create MST nodes for all the word pairs. It creates
;nodes for only those word pairs which have a mutual information not equal to
;-1000
(define (create-MST-nodes text)
	(map create-evaluation-link (filter criteria? (parse text)) ))

;This function is used by the higher-order-function filter to selectively apply
;the create-evaluation-link function to only those word pairs which do not have 
;a mutual information of -1000
(define (criteria? wp)
	(if (= -1000 (get-mutual-information wp))
		#f
		#t))
