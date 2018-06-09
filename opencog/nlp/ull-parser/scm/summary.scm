;
; summary.scm
;
; Print a summary report for a dataset.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; Print a summary report for a dataset.

(use-modules (opencog))
(use-modules (opencog matrix))

; ---------------------------------------------------------------------

(define-public (print-sentence-report)
"
  print-sentence-report - Summarize sentences.
"
	(define n-sent (get-sentence-count))
	(define n-parse (get-parse-count))

	(if (eqv? 0.0 n-sent)
		(throw 'no-sentences 'print-sentence-report
			"Atomspace does not contain any SentenceNodes\n"))

	(format #t "Sentences: ~A     Parses per sentence: ~6f\n"
		n-sent (/ n-parse n-sent))

	(format #t "Unique words: ~A\n" (length (get-all-words)))
)

; ---------------------------------------------------------------------

(define-public (print-word-pair-report)
"
  print-word-pair-report - Summarize properties about word-pairs.
"
	(define any-pairs-obj (make-any-link-api))
	(define wild-obj (add-pair-stars (any-pairs-obj)))
	(define pca (add-pair-count-api wild-obj))

	(define nww (pca 'wild-wild-count))

	; Fetch them, if needed.
	(if (eqv? 0.0 nww)
		(begin
			; XXX FIXME work on the singletons API so that we don't
			; need to fetch the words.
			(display "Start loading words ...\n")
			(call-only-once fetch-all-words)
			(display "Done loading words\n")

			(fetch-atom (any-pairs-obj 'wild-wild))
			(set! nww (pca 'wild-wild-count))))

	(if (eqv? 0.0 nww)
		(throw 'no-word-pairs 'print-word-pair-report
			"Atomspace does not contain any word pairs\n"))

	(format #t "Word pair observations: ~A\n" nww)

	(print-matrix-summary-report any-pairs-obj #t)
)

; ---------------------------------------------------------------------

(define-public (print-language-report)
"
  print-language-report -- print a summary report describing the
  contents of the atomspace.  Intended to summary language-learning
  status.
"
	(catch #t
		(lambda ()
			(print-sentence-report)
		)
		(lambda (key . args)
			(format #t "~A: ~A: ~A \n" key (car args) (cadr args))
			#f
		))
)

; ---------------------------------------------------------------------
