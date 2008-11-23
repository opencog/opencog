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
(define (tot-inflected)
	(define row #f)
	(define row-count 0)
	(define word-count 0)

	(dbi-query db-connection "SELECT count FROM InflectMarginal;")
	(set! row (dbi-get_row db-connection))
	(while (not (equal? row #f))
		(set! word-count (+ word-count 1))
	   (set! row (dbi-get_row db-connection))
	)
	tot-count
	word-count
)

(display (tot-inflected))
