;Author: Rohit Shinde
;Date: 20th July, 2015

;This procedure gets the count of the disjunct
(define (get-disjunct-count disjunct)
	(cdr (cadr (cdr (cog-tv->alist (cog-tv disjunct))))))

;This procedure gets the name and the sign of the MSTConnector once given
;that MSTConnector atom. This returns the list of form (MA.+)
(define (get-connector-name-sign mst-connector)
	(define connector-out-set (cog-outgoing-set mst-connector))
	(define connector-node (car connector-out-set))
	(define direction-node (cadr connector-out-set))
	(define connector-name (cog-name connector-node))
	(define direction-sign (cog-name direction-node))
	(cons connector-name direction-sign))

;This procedure gets all the MSTConnectors for a given disjunct
(define (get-mst-connector disjunct)
	(define disjunct-out-set (cog-outgoing-set disjunct))
	(define mst-connectors (cog-outgoing-set (cadr disjunct-out-set)))
	mst-connectors)

;This procedure takes a direction (+/-) and a set of MSTConnectors in the form
;(MA.+) and gives the number of either + signs or - signs. The
;connector-sign-list is a list of the form ((MA.+) (MB.-) (MD.+)) and so on
(define (get-number-of-direction-signs direction connector-sign-list)
	(define (direction? x)
		(if (equal? x direction)
			#t
			#f))
	(define (get-sign connector-sign)
		(cdr connector-sign))
	(define (get-signs-from-list connector-sign-list)
		(map get-sign connector-sign-list))
	(length (filter  direction? (get-signs-from-list connector-sign-list))))

(define (get-plus-minus-signs connector-sign-list) 
	(cons 
		(cons "+" (get-number-of-direction-signs "+" connector-sign-list)) 
		(cons "-" (get-number-of-direction-signs "-" connector-sign-list))))

;This procedure gets all the disjuncts which have a particular word in them
(define (get-disjunct-for-word disjunct-list word)
	(define disjunct (car disjunct-list))
	(define wordnode (cog-name (car (cog-outgoing-set disjunct))))
	(if (equal? wordnode word)
		(if (not (null? (cdr disjunct-list)))
			(cons disjunct (get-disjunct-for-word (cdr disjunct-list) word))
			'())
		(if (not (null? (cdr disjunct-list)))
			(get-disjunct-for-word (cdr disjunct-list) word)
			'())))

;Given a word, this procedure checks if the given word and the disjunct word
;are the same
(define (disjunct-filter? word disjunct)
	(define wordnode (cog-name (car (cog-outgoing-set disjunct))))
	(if (equal? word wordnode)
		#t
		#f))

;Go through all disjuncts in the list. Find those disjuncts which have the same
;number of plus and minus connectors as the given disjunct. They
;should also have the same wordnode as the given disjunct. Return the number
;of such disjuncts
(define (disjuncts-same-sign-number disjunct disjunct-list)
	(define connector-sign-list (map get-connector-name-sign (get-mst-connector disjunct)))
	(define plus-minus-signs (get-plus-minus-signs connector-sign-list))
	(define num-plus (cdr (car plus-minus-signs)))
	(define num-minus (cdr (cdr plus-minus-signs)))
	(define pm-list (cons num-plus num-minus))
	(define current-disjunct (car disjunct-list))
	(define word (cog-name (car (cog-outgoing-set disjunct))))
	(if (not (null? (cdr disjunct-list)))
		(let* ([csl (map get-connector-name-sign (get-mst-connector current-disjunct))]
			   [pmsigns (get-plus-minus-signs csl)]
			   [np (cdr (car pmsigns))]
			   [nm (cdr (cdr pmsigns))]
			   [pml (cons np nm)]
			   [w (cog-name (car (cog-outgoing-set current-disjunct)))])
			(if (and (equal? w word) (equal? pml pm-list))
				(cons current-disjunct (disjuncts-same-sign-number disjunct (cdr disjunct-list)))
				(disjuncts-same-sign-number disjunct (cdr disjunct-list))))
		(let* ([csl (map get-connector-name-sign (get-mst-connector current-disjunct))]
			   [pmsigns (get-plus-minus-signs csl)]
			   [np (cdr (car pmsigns))]
			   [nm (cdr (cdr pmsigns))]
			   [pml (cons np nm)]
			   [w (cog-name (car (cog-outgoing-set current-disjunct)))])
			(if (and (equal? w word) (equal? pml pm-list))
				current-disjunct
				'()))))

;Get the nth element of a list
(define (nth n l)
  (if (or (> n (length l)) (< n 0))
    (error "Index out of bounds.")
    (if (eq? n 0)
      (car l)
      (nth (- n 1) (cdr l)))))

;Delete nth element of list
(define (delete-n l n)
  (if (= n 0) 
      (cdr l)
      (append (list (car l)) (delete-n (cdr l) (- n 1)))))

;Create a list from 0 to n-1
(define (make-list n)
  (let loop ((n n) (accumulator '()))
    (if (zero? n)
        accumulator
        (loop (- n 1) (cons (- n 1) accumulator)))))

;Get the connector-sign-list like this: (map get-connector-name-sign (get-mst-connector disjunct))
(define (check-if-disjunct-matches-connectors connector-sign-list disjunct wildcard-sign)
	(define (check-if-connectors-present d-csl csl)
		(map (lambda (x) 
				(if (member x d-csl)
					#t
					#f)) csl))
	(define (check-all-true lst)
		(fold-right (lambda (a b) (and a b)) #t lst))
	(define d-csl (map get-connector-name-sign (get-mst-connector disjunct)))
	(define plus-minus-signs-csl (get-plus-minus-signs connector-sign-list))
	(define num-plus-csl (cdr (car plus-minus-signs-csl)))
	(define num-minus-csl (cdr (cdr plus-minus-signs-csl)))
	(define plus-minus-signs-dcsl (get-plus-minus-signs d-csl))
	(define num-plus-dcsl (cdr (car plus-minus-signs-dcsl)))
	(define num-minus-dcsl (cdr (cdr plus-minus-signs-dcsl)))
	(if (equal? wildcard-sign "+")
		(if (and (equal? (+ 1 num-plus-csl) num-plus-dcsl) (equal? num-minus-csl num-minus-dcsl))
			(if (check-all-true (check-if-connectors-present d-csl connector-sign-list))
				#t
				#f)
			#f)
		(if (equal? wildcard-sign "-")
			(if (and (equal? num-plus-csl num-plus-dcsl) (equal? (+ 1 num-minus-csl) num-minus-dcsl))
			(if (check-all-true (check-if-connectors-present d-csl connector-sign-list))
				#t
				#f)
			#f))))

;For a given connector-list and sign, this function returns all the disjuncts
;that match this criteria.
(define (get-number-of-disjuncts disjunct-list connector-sign-list wildcard-sign)
	(define (helper-function disjunct-list connector-sign-list wildcard-sign)
		(if (not (null? (cdr disjunct-list)))
			(cons (check-if-disjunct-matches-connectors connector-sign-list (car disjunct-list) wildcard-sign) 
				(helper-function (cdr disjunct-list) connector-sign-list wildcard-sign))
			'()))
	(count identity (helper-function disjunct-list connector-sign-list wildcard-sign)))

;Given a connector-sign-list ((MA.+) (MB.-) (MC.+)), this function will return
;all possible wildcard configurations. (((MB.-) (MC.+)) +) (((MA.+) (MC.+)) -)
;and so on.
(define (get-all-wildcards connector-sign-list)
	(define len (length connector-sign-list))
	(define (helper-wildcard-function num)
		(define nth-element (nth num connector-sign-list))
		(define sign-of-nth (cdr nth-element))
		(cons (delete-n connector-sign-list num) sign-of-nth))
	(map helper-wildcard-function (make-list len)))

;This function returns a list of numbers for each possibility of the wildcard 
;in the connector-sign-list. This function calculates the denominator of the
;logarithm term.
(define (check-for-all-disjunct-matches connector-sign-list disjunct-list)
	(define wildcard-csl (get-all-wildcards connector-sign-list))
	(define (helper-function wildcard-csl)
		(define wcsl (car wildcard-csl))
		(if (not (null? (cdr wildcard-csl)))
			(let* ([csl (car wcsl)]
				   [sign (cdr wcsl)])
				(list (get-number-of-disjuncts disjunct-list csl sign) (helper-function (cdr wildcard-csl))))
			(let* ([csl (car wcsl)]
				   [sign (cdr wcsl)])
				(get-number-of-disjuncts disjunct-list csl sign))))
	(helper-function wildcard-csl))

;Takes the product of all elements in the list. Even if it is a nested list.
(define (my-product lst)
  (cond
  	([not (pair? lst)] lst)
    ([null? lst] 1)
    ([list? (car lst)] (* (my-product (car lst)) (my-product (cdr lst))))
    (else (* (car lst) (my-product (cdr lst))))))

;This function will calculate denominator of the MI term
(define (get-denominator-for-log connector-sign-list disjunct-list n)
	(define den (expt n (length connector-sign-list)))
	(define num (my-product (check-for-all-disjunct-matches connector-sign-list disjunct-list)))
	(/ num den))

;function to get log in another base
(define logB (lambda (x B) 
      (/ (log x) (log B))))

;This function calculates the numerator for the MI formula
(define (get-numerator-for-log disjunct disjunct-list)
	(/ (get-disjunct-count disjunct) (length (disjuncts-same-sign-number disjunct disjunct-list))))

;This function will calculate the mutual information for the given disjunct
(define (get-mi disjunct disjunct-list)
	(define disjunct-word (cog-name (car (cog-outgoing-set disjunct))))
	(define (disjunct-filter? x)
		(if (equal? disjunct-word (cog-name (car (cog-outgoing-set x))))
			#t
			#f))
	(define filtered-list (filter disjunct-filter? disjunct-list))
	(define connector-sign-list (map get-connector-name-sign (get-mst-connector disjunct)))
	(define numer (get-numerator-for-log disjunct disjunct-list))
	(define denom (get-denominator-for-log connector-sign-list disjunct-list (get-disjunct-count disjunct)))
	(define division (/ numer denom))
	(define logarithm (logB division 2))
	(- logarithm))

;Do this for all disjuncts
(define (do-em-all-disjuncts disjunct-list)
	(map (lambda (x) (get-mi x disjunct-list)) disjunct-list))

