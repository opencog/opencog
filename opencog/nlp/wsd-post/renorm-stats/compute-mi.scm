#! /usr/bin/env guile
!#
;
; compute-mi.scm
;
; Compute the mutual information for the link-grammar disjuncts.
;
; Compute the conditional probability for assorted link-grammar
; disjunct tables. This assumes that the tables have already been
; populated with raw data, and thus, the raw count fields have
; meaningful (and final) values. This script merely totals up the
; counts, and computes the various probabilities.
;
; This script must be run in order to compute the Disjunct tables that
; are used by the link-grammar "corpus" backend, which provides parse
; ranking as well as parse-guiding for the SAT solver.
;
; Currently takes about 100 minutes to run.
;
; Copyright (c) 2008,2009 Linas Vepstas <linasvepstas@gmail.com>
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
(define db-disjunct-conn
	(dbi-open "postgresql" "linas:asdf:lexat:tcp:localhost:5432"))
(define db-update-conn
	(dbi-open "postgresql" "linas:asdf:lexat:tcp:localhost:5432"))
;; (define db-debug-conn
;; 	(dbi-open "postgresql" "linas:asdf:lexat:tcp:localhost:5432"))

;; Provide the names of the tables that hold the actual data.
;;
;; (define inflected-tablename "InflectMarginal")
;; (define disjuncts-tablename "Disjuncts")

(define inflected-tablename "NewInflectMarginal")
(define disjuncts-tablename "NewDisjuncts")

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

	(dbi-query db-select-conn 
		(string-append "SELECT count FROM " inflected-tablename ";")
	)
	(set! row (dbi-get_row db-select-conn))
	(while (not (equal? row #f))
		(set! row-count (+ row-count 1))
		(set! word-count (+ word-count (assoc-ref row "count")))
		(set! row (dbi-get_row db-select-conn))
	)
	word-count
)

;; --------------------------------------------------------------------
;; debugging utility
;;
(define (dbg-approx)
	(define rc #f)
	(define row #f)

	(dbi-query db-debug-conn 
		"SELECT log_probability FROM InflectMarginal WHERE inflected_word = 'approx[?].v';"
	)
	(set! row (dbi-get_row db-debug-conn))
	(while (not (equal? row #f))
		(if (< 22 (assoc-ref row "log_probability"))
			(set! rc #t)
		)
		(set! row (dbi-get_row db-debug-conn))
	)
	rc
)

;; --------------------------------------------------------------------
;;
;; Compute and store the marginal word probability for a word.
;; Divide the count for each row by a sum-total of all counts.
;; 
;; Arguments: word, count for this word, total count.
;;
(define (update-marginal-probability word count-bare tot)
	(define urow #f)

	(let* ((count (if (< 0 count-bare) count-bare 1.0e-30))
				(lprob (/ (- (log (/ count tot))) (log 2)))
				(sprob (number->string lprob))
			)
		(dbi-query db-update-conn 
			(string-append "UPDATE " inflected-tablename 
				" SET log_probability = "
				sprob " WHERE inflected_word = $$" word "$$"
			)
		)

		;; Call twice -- once to update, once to flush connection
		(set! urow (dbi-get_row db-update-conn))
		(set! urow (dbi-get_row db-update-conn))
	)
)

;; --------------------------------------------------------------------
;;
;; Compute and store the marginal word probabilities
;; This is done by going through the InflectMarginal table, and dividing
;; the count for each row by the sum-total of all counts in the table.
;;
(define (marginal-set-probabilities)
	(define cnt 0)
	(define srow #f)
	(define tot (marginal-tot-inflected))

	(dbi-query db-select-conn 
		(string-append 
			"SELECT inflected_word, count FROM " inflected-tablename ";"
		)
	)

	; Loop over all words in the database.
	(set! srow (dbi-get_row db-select-conn))
	(while (not (equal? srow #f))
		(let* ((word (assoc-ref srow "inflected_word"))
				(wcount(assoc-ref srow "count"))
			)
			;; update the table row.
			(update-marginal-probability word wcount tot)
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
	(display " Tot weight: ")
	(display tot)
	(newline)
)

;; --------------------------------------------------------------------
;;
;; Update the conditional probability for a single inflected word.
;; This will loop over all (word,disjunct) pairs, computing the 
;; conditional probability of seeing that disjunct, given that the
;; word is being observed.
;;
;; Arguments: the word, and the number of times its been observed.
;;
(define (update-disjunct-cond-probability word word-cnt)
	(define drow #f)
	(define urow #f)

	; Use the postgres $$ string delimiter -- the problem is that
	; some of the database entries contain back-slashes, which are
	; not escapes.  Treating them as escapes leads to big trouble.
	(dbi-query db-disjunct-conn 
		(string-append "SELECT count, disjunct FROM "
			disjuncts-tablename
			" WHERE inflected_word = $$" word "$$"
		)
	)
	(set! drow (dbi-get_row db-disjunct-conn))
	(while (not (equal? drow #f))
		(let* ((dj-cnt-bare (assoc-ref drow "count"))
				(dj (assoc-ref drow "disjunct"))
				(dj-cnt (if (< 0 dj-cnt-bare) dj-cnt-bare 1.0e-30))
				(dprob (/ (- (log (/ dj-cnt word-cnt))) (log 2)))
				(sdprob (number->string dprob))
			)
			(dbi-query db-update-conn 
				(string-append "UPDATE "
					disjuncts-tablename
					" SET log_cond_probability = "
					sdprob " WHERE inflected_word = $$" word 
					"$$ AND disjunct = '" dj "'"
				)
			)

			;; Call twice -- once to update, once to flush connection
			(set! urow (dbi-get_row db-update-conn))
			(set! urow (dbi-get_row db-update-conn))
		)
		(set! drow (dbi-get_row db-disjunct-conn))
	)
)

;; --------------------------------------------------------------------
;;
;; Compute the conditional probabilites for the disjunct table.
;; That is, given a particular (word,disjunct) pair, compute the 
;; conditional probability of seeing that disjunct, given that the
;; word is being observed.
;;
(define (disjunct-cond-probabilities)
	(define cnt 0)
	(define srow #f)

	(dbi-query db-select-conn 
		(string-append 
			"SELECT inflected_word, count FROM " inflected-tablename ";"
		)
	)

	; Loop over all words in the database.
	(set! srow (dbi-get_row db-select-conn))
	(while (not (equal? srow #f))

		; Do a single word.
		(update-disjunct-cond-probability 
			(assoc-ref srow "inflected_word")
			(assoc-ref srow "count")
		)

		; Get the next row.
		(set! srow (dbi-get_row db-select-conn))

		; Print a running total, since this takes a long time.
		(set! cnt (+ cnt 1))
		(if (eq? 0 (modulo cnt 1000))
			(let ()
				(display "finished disjuncts for ")
				(display cnt)
				(display " words\n")
			)
		)
	)
	(display "Total disjuncts: ")
	(display cnt)
	(newline)
)

;; --------------------------------------------------------------------
;;
;; Compute the marginal probabilities
(display "Start computing the marginal probabilities\n")
(system "date")
(marginal-set-probabilities)
(display "Done computing the marginal probabilities\n")
(system "date")
;
(disjunct-cond-probabilities)
(display "Done computing the conditional probabilities\n")
(system "date")
