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

;; --------------------------------------------------------------------
;; cluster.
;;
(define (cluster)
	(define cnt 0)
	(define srow #f)

	;; Loop over all words in the database
	(dbi-query db-select-conn 
		"SELECT inflected_word FROM InflectMarginal;"
	)

	; Loop over all words in the database.
	(set! srow (dbi-get_row db-select-conn))
	(while (not (equal? srow #f))
		(let* ((word (assoc-ref srow "inflected_word"))
			)
			;; cluster the word
			(cluster-word word)
		)

		; get the next row
		(set! srow (dbi-get_row db-select-conn))

		; print a running total, since this takes a long time.
		(set! cnt (+ cnt 1))
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

