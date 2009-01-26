#! /usr/bin/guile
!#
;;
;; cluster.scm
;; Low-brow data cluster analysis program.
;;
;; Copyright (C) 2009 Linas Vepstas <linasvepstas@gmail.com>
;; 

(use-modules (srfi srfi-1))
(use-modules (dbi dbi))

;; username is 'linas' in this example, passwd is 'asdf' and the
;; databasse is 'lexat', running on the local machine
;; The postgres server is at port 5432, which can be gotten from
;; /etc/postgresql/8.3/main/postgresql.conf
;; 
(define db-select-conn
	(dbi-open "postgresql" "linas:asdf:lexat:tcp:localhost:5432"))
(define db-dj-conn
	(dbi-open "postgresql" "linas:asdf:lexat:tcp:localhost:5432"))

;; cluster radius
(define cluster-radius 3.0)

;; minimum number of times that a word must have been observed.
(define min-observed-weight 1.0)

;; max distance beyond which a point is "at infinity"
(define max-distance 5.0)

;; --------------------------------------------------------------------
;; Global variable holding list of clusters
;;
(define cluster-list '())

;; --------------------------------------------------------------------
;; create a new cluster, return it
;;
(define (new-cluster word coords)
	(list
		(cons "wordlist" (cons word '()))
		(cons "center" coords)
		(cons "npts" 1)
	)
)

;; --------------------------------------------------------------------
;; add to an existing cluster
;;
(define (add-to-cluster cluster word coords)

	;; prepend the word to the cluster's word-list.
	(set! cluster 
		(assoc-set! cluster "wordlist"
			(cons word (assoc-ref cluster "wordlist"))
		)
	)

	;; average in the center.
	;; update the count
	(set! cluster 
		(assoc-set! cluster "npts"
			(+ 1 (assoc-ref cluster "npts"))
		)
	)
	cluster
)

;; --------------------------------------------------------------------
;; note a new cluster
;;
(define (note-cluster cluster)
	(show-cluster cluster)
	(set! cluster-list (cons cluster cluster-list))
)

;; --------------------------------------------------------------------
;; show a cluster
;;
(define (show-cluster cluster)
	(display "word is ")
	(display (assoc-ref cluster "wordlist"))
	(newline)
	(display "coords are ")
	(display (assoc-ref cluster "center"))
	(newline)
)

;; --------------------------------------------------------------------
;; return true if a coordinate point is in a cluster, else return false
;;
(define (in-cluster? cluster coords)
	#f
)

;; --------------------------------------------------------------------
;; return the cluster that a point belongs to, else return nil
;;
(define (find-cluster clist coords)
	(if (null? clist)
		clist
		(if (in-cluster? (car clist) coords) 
			(car clist)
			(find-cluster (cdr clist) coords)
		)
	)
)

;; --------------------------------------------------------------------
;; assign single word to a cluster
;;
;; The word coordinates are based on the disjunct's conditional probabilities.
;; Look these up in the database.
;;
(define (cluster-word word)
	(define row #f)
	(define coords '())

	; look up word in disjunts table
	(dbi-query db-dj-conn 
		(string-append
			"SELECT disjunct, log_cond_probability FROM Disjuncts WHERE inflected_word=$$"
			word "$$"
		)
	)
	
	; The "coordinates" of the word are simply a list of disjuncts
	; and thier log cond probs.
	(set! row (dbi-get_row db-dj-conn))
	(while (not (equal? row #f))
		(let* ((dj (assoc-ref row "disjunct"))
				(lcp (assoc-ref row "log_cond_probability"))
			)
			; record a distance, but only if its not far away.
			(if (< lcp max-distance)
				(set! coords (cons (cons dj lcp) coords))
			)
		)
		(set! row (dbi-get_row db-dj-conn))
	)

	; Find the cluster
	(let* ((cluster (find-cluster cluster-list coords)))
		(if (null? cluster)
			(note-cluster (new-cluster word coords))
			(add-to-cluster word coords)
		)
	)
)

;; --------------------------------------------------------------------
;; cluster.
;;
(define (cluster)
	(define cnt 0)
	(define srow #f)

	;; Loop over all words in the database
	(dbi-query db-select-conn 
		"SELECT inflected_word, count FROM InflectMarginal;"
	)

	; Loop over all words in the database.
	(set! srow (dbi-get_row db-select-conn))
	(while (not (equal? srow #f))
		(let* ((word (assoc-ref srow "inflected_word"))
				(word-count (assoc-ref srow "count"))
			)
			;; Cluster the word, but only if it is a word that 
			;; link-grammar handled (i.e. is not in square brackets)
			;; and only if it has been observed a few times -- having
			;; a count that is more than "min-observed-weight"
			(if (and 
					(not (equal? #\[ (string-ref word 0)))
					(< min-observed-weight word-count)
					)
				(let ()
					(cluster-word word)
					(set! cnt (+ cnt 1))
				)
			)
		)

		; get the next row
		(set! srow (dbi-get_row db-select-conn))

		; print a running total, since this takes a long time.
		(if (eq? 0 (modulo cnt 1000))
			(let ()
				(display cnt)
				(display " words processed\n")
			)
		)
	)

	(display "Inflected words: ")
	(display cnt)
	(newline)
)

;; --------------------------------------------------------------------
;; do it
(cluster)
