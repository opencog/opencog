;
; export-mi.scm
;
; The hacky and ugly code below looks for every EvaluationLink
; containing a ListLink with two WordNodes in the atomspace,
; then prints that word-pair's FMI to file.
; For research purposes only.
;
;

(use-modules (opencog sheaf))

(define-public (export-mi cnt-mode)
"
  Export the word-pairs fMI for
  every possible pair in the corpus, to allow analysis
  of the MI-based MST-parser

  The format is:
  word#1 word#2 pair-mi
  word#5 word#3 pair-mi
  ...
"
	; uncomment for LG-any counting
        ;(define pair-obj (make-any-link-api))
	; uncomment for clique-pair-counting
        (define pair-obj (if (equal? cnt-mode "any")
				(make-any-link-api)
				(make-clique-pair-api)))
        (define mi-source (add-pair-freq-api pair-obj))
        (define scorer (make-score-fn mi-source 'pair-fmi))

        (define file-port-fmi (open-file "mi-pairs.txt" "w"))

        ; Print an mi-pair
        (define (print-mi EL)
                (let ([l-node (gar (gdr EL))] [r-node (gdr (gdr EL))])
                        (if
                                (and
                                        (equal? 'WordNode (cog-type l-node))
                                        (equal? 'WordNode (cog-type r-node)))
                                (display
                                        (format #f "~a ~a ~a\n"
                                                (cog-name l-node)
                                                (cog-name r-node)
                                                (scorer l-node r-node 1))
                                        file-port-fmi))))

        ; Print MI-info
        (for-each
                (lambda (e)
                        (print-mi e))
                (cog-get-atoms 'EvaluationLink))

        (close-port file-port-fmi)
)
