;
; pair-object-api.scm
;
; Define OO API's for pairs of things.
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
; Example low-level API object. It has only five methods; these
; return pair-atoms on which counts are stored as values.
; Higher-evel objects use this object to fetch counts, store them
; into the database, or to return various statistics.
;
; See `make-any-link` for a working example.
;
;  (define (make-ll-object-api-example)
;     (let ()
;        ; Return the atom-type of the left and right items.
;        ; For example both may be words, or maybe the right
;        ; side is a disjunct 'LgAnd
;        (define (get-left-type) 'WordNode)
;        (define (get-right-type) 'WordNode)
;
;        ; Return the atom holding the count.
;        (define (get-pair PAIR) "foobar")
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
;     ; Methods on the object. To call these, quote the method name.
;     ; Example: (OBJ 'get-left-wildcard WORD) calls the
;     ; get-left-wildcard method, passing WORD as the argument.
;     (lambda (message . args)
;        (apply (case message
;              ((left-type) get-left-type)
;              ((right-type) get-right-type)
;              ((item-pair) get-pair)
;              ((left-wildcard) get-left-wildcard)
;              ((right-wildcard) get-right-wildcard)
;              ((wild-wild) get-wild-wild)
;              (else (error "Bad method call on low-level API")))
;           args))))
;
;
; ---------------------------------------------------------------------
;
;
(define (make-pair-object-api-example)
	(let ()
		; Return the atom holding the count.
		(define (get-pair PAIR) "foobar")

		; Return a list of atoms hold the count.
		(define (get-pairs PAIR) "foobar")

		(define (get-left-wildcard WORD) "foobar")

		(define (get-right-wildcard WORD) "foobar")

		(define (get-wild-wild) "foobar")

	; Methods on the object
	; Example: (OBJ 'get-left-wildcard WORD) calls the
	; get-left-wildcard method, passing WORD as the object.
	(lambda (message . args)
		(apply (case message
				((get-pair) get-pair)
				((get-left-wildcard) get-left-wildcard)
				((get-right-wildcard) get-right-wildcard)
				((get-wild-wild) get-wild-wild)
				(else (error "Bad method call on ANY-link")))
			args))))
