(define (show-cnts)
  (define (cnt-type ty)
    (let ((cnt 0))
     (define (incrm a) (set! cnt (+ cnt 1)) #f)
     (cog-map-type incrm ty)

     ; print only the non-zero counts
     (if (< 0 cnt)
       (begin
         (display ty)
         (display "  ")
         (display cnt)
         (newline)
         )
       )
     )
    )
  (for-each cnt-type (cog-get-types))
  )
