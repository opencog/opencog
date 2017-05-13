;
; pair-object-api.scm
;
; Define object-oriented class API's for pairs of things.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; In this project, there's a generic theme of "pairs of things" that
; are statistically related. The can be pairs of words, they can be
; connector-sets, which are a pair of (word, disjunct), or they can
; be other things.
;
; For all of these pairs (x,y), we typically need to get the count
; N(x,y), the partial sums N(x,*) = sum_y N(x,y), and likewise N(*,y)
; and N(*,*).   We need to compute frequencies of observations, such
; as p(x,y) = N(x,y)/N(*,*).  We also need to compute entropies and
; mutual information, which can be infered from these frequencies.
; We also can compute cosine-similairy and other matrics of similarity,
; dervied solely from the observed frequency counts.
;
; All of these formulas are independent of the actual objects in the
; pairs.  Thus, it is useful to separae the various algorithms from
; the data that they operate on. Towards this end, this file defines
; some object-oriented OO API's for pairs, which the algos can assume,
; and the different types of pairs can implement.
;
; There are several API's here. The lowest-level ones are listed first.
;
; The GET-PAIR argument should be a function that returns all atoms
; that hold counts for a pair (ListLink left-item right-item). It
; should return a list (possibly the empty list).  Note that in some
; cases, the list may be longer than just one item, e.g. for schemas
; that count link lengths.  In thise case, the total count for the
; pair is taken to be the sum of the individual counts on all the
; atoms in the list.


; ---------------------------------------------------------------------
;
; Example low-level API class. It has only six methods; these
; return pair-atoms on which counts are stored as values.
; Higher-evel objects use this object to fetch counts, store them
; into the database, or to return various statistics.
;
; The `make-pair-count-get-set` class, below, is a typical user
; of this class; it provides getters and setters for teh counts.
;
; See `make-any-link` for a working example.
;
; When called, this will create a new instance of the class
; i.e. will create a new object.
;
;  (define (make-ll-object-api-example)
;     (let ()
;        ; Return the atom-type of the left and right items.
;        ; For example both may be words, or maybe the right
;        ; side is a disjunct 'LgAnd
;        (define (get-left-type) 'WordNode)
;        (define (get-right-type) 'WordNode)
;
;        ; Return the atom holding the count, if it exists,
;        ; else return nil.
;        (define (get-pair PAIR) "foobar")
;
;        ; Return the atom holding the count, creating it if
;        ; it does not yet exist.
;        (define (make-pair PAIR) "foobar")
;
;        ; Return a list of atoms hold the count.
;        (define (get-pairs PAIR) "foobar")
;
;        ; Return the atom holding the N(*,y) count
;        (define (get-left-wildcard ITEM) "foobar")
;
;        ; Return the atom holding the N(x,*) count
;        (define (get-right-wildcard ITEM) "foobar")
;
;        ; Return the atom holding the N(*,*) count
;        (define (get-wild-wild) "foobar")
;
;     ; Methods on the class. To call these, quote the method name.
;     ; Example: (OBJ 'left-wildcard WORD) calls the
;     ; get-left-wildcard method, passing WORD as the argument.
;     (lambda (message . args)
;        (apply (case message
;              ((left-type) get-left-type)
;              ((right-type) get-right-type)
;              ((item-pair) get-pair)
;              ((make-pair) make-pair)
;              ((left-wildcard) get-left-wildcard)
;              ((right-wildcard) get-right-wildcard)
;              ((wild-wild) get-wild-wild)
;              (else (error "Bad method call on low-level API")))
;           args))))
;
;
; ---------------------------------------------------------------------
;
; Extend the LLOBJ with additional methods to get and set
; various values on the objects.
;
(define (make-pair-count-get-set LLOBJ)
	(let ((llobj LLOBJ))

		; Return the raw observational count on PAIR.
		; If the PAIR does not exist (was not oberved) return 0.
		(define (get-pair-count PAIR)
			(define pr (llobj 'item-pair PAIR))
			(if (null? pr) 0
				(cog-tv-count (cog-tv pr))))

		; Set the raw observational count on PAIR
		(define (set-pair-count PAIR CNT)
			(cog-set-tv! (llobj 'make-pair PAIR) (cog-new-ctv 0 0 CNT)))

		; Get the left wildcard count
		(define (get-left-wild-count ITEM)
			(get-pair-count (llobj 'left-wildcard ITEM)))

		; Get the right wildcard count
		(define (get-right-wild-count ITEM)
			(get-pair-count (llobj 'right-wildcard ITEM)))

		; Set the left wildcard count
		(define (set-left-wild-count ITEM CNT)
			(set-pair-count (llobj 'left-wildcard ITEM) CNT))

		; Set the right wildcard count
		(define (set-right-wild-count ITEM CNT)
			(set-pair-count (llobj 'right-wildcard ITEM) CNT))

		; Get the wildcard-wildcard count
		(define (get-wild-wild-count)
			(get-pair-count (llobj 'wild-wild)))

		; Set the wildcard-wildcard count
		(define (set-wild-wild-count CNT)
			(set-pair-count (llobj 'wild-wild) CNT))


	; Methods on this class.
	(lambda (message . args)
		(apply (case message
				((pair-count) get-pair-count)
				((set-pair-count) set-pair-count)
				((left-wild-count) get-left-wild-count)
				((set-left-wild-count) set-left-wild-count)
				((right-wild-count) get-right-wild-count)
				((set-right-wild-count) set-right-wild-count)
				((wild-wild-count) get-wild-wild-count)
				((set-wild-wild-count) set-wild-wild-count)
				(else (llobj message)))
			args))))
