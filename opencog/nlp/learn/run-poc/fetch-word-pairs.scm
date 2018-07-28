;
; Provides the pair-counts for us
; Takes as input the counting mode: lg, clique, clique-distance
;
; NOTE: This script assumes that the databas already contains 
; the mutual information for the pairs obtained from an observe
; pass over the same sentences to be parsed.
;
;  Written by glicerico

(use-modules (opencog) (opencog persist) (opencog persist-sql))
(use-modules (opencog nlp))
(use-modules (opencog matrix))

(define (fetch-wp cnt-mode)
  (define pair-obj '())
  (define star-obj '())
  (display "Fetching all word-pairs from the database. This may take a few minutes.\n")
  (cond
    ((equal? cnt-mode "lg")
      (set! pair-obj (make-any-link-api)))

    ((or (equal? cnt-mode "clique")
         (equal? cnt-mode "clique-dist"))
      (set! pair-obj (make-clique-pair-api))))

  (set! star-obj (add-pair-stars pair-obj)) ;TODO: Can it be left out??
  (pair-obj 'fetch-pairs)

  ; Print the sql stats
  (sql-stats)

  ; Clear the sql cache and the stats counters
  (sql-clear-cache)
  (sql-clear-stats)

  (display "Done fetching pairs.\n")
)
