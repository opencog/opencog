;Author: Rohit Shinde
;Date: 29th May, 2015

;This file defines all the functions needed to extract the word pairs having
;the highest mutual information. I have hardcoded the information for 
;top 50 word pairs with the highest mutual information, but this can be
;extended to any number.
(define (create-conf-list lst)
  (define (helper ls acc)
    (define wp (car ls))
    (define confidence (tv-conf (cog-tv wp)))
    (if (null? (cdr ls))
        (reverse acc)
      	(helper (cdr ls) (cons confidence acc))))
    (helper lst '()))

(define LGNode (LinkGrammarRelationshipNode "ANY"))

(define wordpairs (cog-incoming-set LGNode))

(define num-of-wordpairs (length wordpairs))

(define mi-num (create-conf-list wordpairs))

(define sorted-mi (sort mi-num >))

(define top-50-mi (take sorted-mi 50))

(define (get-wordpair mi wordpairs)
	(define current-wp (car wordpairs))
	(define confidence (tv-conf (cog-tv current-wp)))
	(if (equal? confidence mi)
		current-wp
		(get-wordpair mi (cdr wordpairs))))

;delete from a sorted list
(define (delete1! v l)
  (let loop ((left '()) (right l))
    (if (null? right) l
        (if (equal? (car right) v) 
            (if (null? left) (cdr right)
                (begin (set-cdr! left (cdr right)) 
                       l))
            (loop right (cdr right))))))

(define (mi-filter mi wordpairs) (filter (lambda (e)
          (equal? (tv-conf (cog-tv e)) mi))
        wordpairs))

;This function will remove duplicate elements from a list.
(define (remove-duplicated lst)
  (if (or (null? lst) (null? (cdr lst))) ; changes here
      lst
      (if (= (car lst) (cadr lst))
          (remove-duplicated (cdr lst))  ; and here
          (cons (car lst)
                (remove-duplicated (cdr lst))))))

(define unique-mi (remove-duplicated top-50-mi))

;gets the top 50 wordpairs
(define top-50-wp (map mi-filter unique-mi))

;Get the frequency of the word pair
(define (get-count wordpair)
	(cdr (car (cddr (cog-tv->alist (cog-tv wordpair))))))

(define (get-mi wp)
	(tv-conf (cog-tv wp)))

(define sorted-wp (sort wordpairs (lambda (x y) (> (get-mi x) (get-mi y)))))

(define (count-filter? x)
  (define count (get-count x))
  (if (< count 1000)
      #f
      #t))

(define (attach-count-mi wp)
	(define mi (get-mi wp))
	(define count (get-count wp))
	(cons wp (cons mi count)))

(define (map-proc wp) (map attach-count-mi wp))

(define final (map map-proc top-50-wp))

(define (predicate? x)
	(define count (cdr (cdr x)))
	(if (> count 100)
		#t
		#f))

(define (filter-count cnt-mi-lst) (filter predicate? cnt-mi-lst))

(map filter-count final)

(define (sort-desc-by-second lst-of-wp)
  (sort lst
        (lambda (x y) (> (cadr x) (cadr y)))))

(define (delete-n-elements n lst)
  (if (> n 0)
      (delete-n-elements (- n 1) (cdr lst))
      lst))

