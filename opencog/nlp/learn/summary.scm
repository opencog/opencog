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
(use-modules (opencog analysis))

; ---------------------------------------------------------------------

(define-public (print-sentence-report)
"
  print-sentence-report - Summarize sentences.
"
	(define n-sent (get-sentence-count))
	(define n-parse (get-parse-count))

	(if (eqv? 0 n-sent)
		(throw 'no-sentences 'print-sentence-report
			"Atomspace does not contain any SentenceNodes\n"))

	(format #t "Sentences: ~A     Parses per sentence: ~6f\n"
		n-sent (/ n-parse n-sent))
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
