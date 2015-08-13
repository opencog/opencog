;Author: Rohit Shinde
;Date: 27th May, 2015

;This file has the functions which will sort all wordpairs according to their
;frequency or their ctv. Then the wordpairs are removed, and only the frequency
;numbers remain. This output is then sent to a file where it is plotted on a 
;graph.
;Use like this: (filter filter-anyword wordpairs)
(define (filter-anyword wp)
	(define outgoing-set-wp (cog-outgoing-set wp))
	(define listlink (cadr outgoing-set-wp))
	(define outlistlink (cog-outgoing-set listlink))
	(define firstnodetype (cog-type (car outlistlink)))
	(define secondnodetype (cog-type (cadr outlistlink)))
	(if (eq? 'AnyNode firstnodetype)
		#f
		(if (eq? 'AnyNode secondnodetype)
			#f
			#t)))

(define (get-frequency-wp wp)
	(cdr (cadr (cdr (cog-tv->alist (cog-tv wp))))))

;The below function will sort the wordpairs according to the frequency.
(define (sorted-freq lst-wp) 
	(sort lst-wp 
		(lambda (x y) (> (get-frequency-wp x) (get-frequency-wp y)))))

(define (create-freq-list lst)
  (define (helper ls acc)
    (define wp (car ls))
    (define freq (get-frequency-wp wp))
    (if (null? (cdr ls))
        (reverse acc)
      	(helper (cdr ls) (cons freq acc))))
    (helper lst '()))

(define (greater-than? x wordpair)
	(define count (cdr (car (cddr (cog-tv->alist (cog-tv wordpair))))))
	(if (> count x)
		#t
		#f))

