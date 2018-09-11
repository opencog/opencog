;
; Computes the mutual information for the word pairs.
; Takes as input the counting mode: lg, clique, clique-distance
(use-modules (opencog) (opencog persist) (opencog persist-sql))
(use-modules (opencog nlp) (opencog nlp learn))
(use-modules (opencog matrix))

(define (comp-mi cnt-mode)
  (define pair-obj '())
  (define star-obj '())
  (cond
    ((equal? cnt-mode "any")
    	(set! pair-obj (make-any-link-api)))

    ((or (equal? cnt-mode "clique")
         (equal? cnt-mode "clique-dist"))
        (set! pair-obj (make-clique-pair-api))))

  (set! star-obj (add-pair-stars pair-obj))
  (batch-pairs star-obj)
  
  ; Print the sql stats
  (print-matrix-summary-report star-obj)

  ; Clear the sql cache and the stats counters
  (sql-close)
 
  (display "Done computing MI from word pairs.\n")
)
