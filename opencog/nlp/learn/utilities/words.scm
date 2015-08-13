;Author: Rohit Shinde
;Date: 28th May, 2015

;This file has all the functions for the following task: Find the nth word
;and get the number of times that it has occurred in word pairs.
;A graph is plotted for this. 
;This is sampled for the formula 2^(k/6).
;For more points, the formula 2^(k/23) can also be used.

(define words (cog-get-atoms 'WordNode))

(define (create-freq-list word-lst)
  	(define (helper ls acc)
    	(define word (car ls))
    	(define count (cdr (cadr (cdr (cog-tv->alist (cog-tv word))))))
    	(if (null? (cdr ls))
	        (reverse acc)
      		(helper (cdr ls) (cons count acc))))
	(helper word-lst '()))

;Get the Nth element from the list
(define (nth n l)
  (if (or (> n (length l)) (< n 0))
    (error "Index out of bounds.")
    (if (eq? n 0)
      (car l)
      (nth (- n 1) (cdr l)))))

(define frequencies (create-freq-list words))

(define sorted-frequencies (sort > frequencies))

(define (get-count word) (cdr (cadr (cdr (cog-tv->alist (cog-tv word))))))

(define sorted-words (sort words (lambda (x y) (> (get-count x) (get-count y)))))

(define (get-num-wordpairs word lst-wp length)
	(define wp (car lst-wp))
	(define firstWordNode (car (cog-outgoing-set (cadr (cog-outgoing-set wp)))))
	(define secondWordNode (cadr (cog-outgoing-set (cadr (cog-outgoing-set wp)))))
	(if (equal? word firstWordNode)
		(if (null? lst-wp)
			length
			(get-num-wordpairs word (cdr lst-wp) (+ 1 length)))
		(if (equal? word secondWordNode)
			(if (null? lst-wp)
				length
				(get-num-wordpairs word (cdr lst-wp) (+ 1 length)))
			(if (null? (cdr lst-wp))
				length
				(get-num-wordpairs word (cdr lst-wp) length)))))

(define (top-words-total words wordpairs)
	(if (not (null? words))
		(cons (get-num-wordpairs (car words) wordpairs 0) (top-words-total (cdr words) wordpairs))
		'()))

(define (get-integer-value-for-expression k)
	(inexact->exact (floor (expt 2 (/ k 6)))))

;This function will create a list of integers from first-last
(define (range first last)
  (if (>= first last)
      '()
      (cons first (range (+ first 1) last))))

(define (get-word-counts n-lst word-lst wp-lst)
	(if (not (null? n-lst))
		(cons (get-num-wordpairs (nth (car n-lst) sorted-words) wp-lst 0) (get-word-counts (cdr n-lst) word-lst wp-lst))
		'()))

