#! /usr/bin/guile
!#
;
; compute-mi.scm
;
; Compute the mutual information for the link-grammar disjuncts
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
(define db-connection
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

	(dbi-query db-connection "SELECT count FROM InflectMarginal;")
	(set! row (dbi-get_row db-connection))
	(while (not (equal? row #f))
		(set! row-count (+ row-count 1))
		(set! word-count (+ word-count (assoc-ref row "count")))
	   (set! row (dbi-get_row db-connection))
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
	(define row #f)
	(define tot (marginal-tot-inflected))

	(dbi-query db-connection "SELECT inflected_word, count FROM InflectMarginal;")
	(set! row (dbi-get_row db-connection))
	(while (not (equal? row #f))
(display (assoc-ref row "inflected_word"))
(display " ")
		(display (/ (assoc-ref row "count") tot)
) (newline)
	   (set! row (dbi-get_row db-connection))
	)

)

(marginal-set-probabilities)
