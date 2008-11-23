#! /usr/bin/guile
!#
;
; compute-mi.scm
;
; Compute the mutual information for the link-grammar disjuncts
;
; Compute the conditional probability for assorted link-grammar
; disjunct tables. This assumes that the tables have already been
; populated with raw data, and thus, the raw count fields have
; meaningful (and final) values. This script merely totals up the
; counts, and computes the various probabilities.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
(use-modules (srfi srfi-1))
(use-modules (dbi dbi))
;
;; username is 'linas' in this example, passwd is 'asdf' and the
;; databasse is 'lexat', running on the local machine
;; The postgres server is at port 5432, which can be gotten from
;; /etc/postgresql/8.3/main/postgresql.conf
;;
(define db-select-conn
   (dbi-open "postgresql" "linas:asdf:lexat:tcp:localhost:5432"))
(define db-update-conn
   (dbi-open "postgresql" "linas:asdf:lexat:tcp:localhost:5432"))

;; --------------------------------------------------------------------
;;
;; Sum up the total number of words observed; and return that total.
;; This is done by going through the InflectMarginal table, and adding
;; up the "count" column.
;;
(define (marginal-tot-inflected)
	(define row #f)
	(define row-count 0)
	(define word-count 0)

	(dbi-query db-select-conn "SELECT count FROM InflectMarginal;")
	(set! row (dbi-get_row db-select-conn))
	(while (not (equal? row #f))
		(set! row-count (+ row-count 1))
		(set! word-count (+ word-count (assoc-ref row "count")))
	   (set! row (dbi-get_row db-select-conn))
	)
	word-count
)

;; --------------------------------------------------------------------
;;
;; Compute and store the marginal word probabilities
;; This is done by going through the InflectMarginal table, and dividing
;; the count for each row by the sum-total of all counts in the table.
;;
(define (marginal-set-probabilities)
	(define srow #f)
	(define urow #f)
	(define tot (marginal-tot-inflected))

	(dbi-query db-select-conn 
		"SELECT inflected_word, count FROM InflectMarginal;"
	)

	; Loop over all words in the database.
	(set! srow (dbi-get_row db-select-conn))
	(while (not (equal? srow #f))
		(let* ((word (assoc-ref srow "inflected_word"))
				(lprob (- (log (/ (assoc-ref srow "count") tot))))
				(sprob (number->string lprob))
				)
			(dbi-query db-update-conn 
				(string-append "UPDATE InflectMarginal SET log_probability = "
					sprob " WHERE inflected_word = E'" word "'"
				)
			)
			;; Call twice -- once to update, once to flush connection
			(set! urow (dbi-get_row db-update-conn))
			(set! urow (dbi-get_row db-update-conn))
		)
	   (set! srow (dbi-get_row db-select-conn))
	)
)

(marginal-set-probabilities)
