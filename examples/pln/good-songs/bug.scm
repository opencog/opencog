(define* (foo mandatory-arg #:key (optional-arg 0))
  (display "foo\n")
  (display mandatory-arg)
  (display "\n")
  (display optional-arg)
  (display "\n"))

(define (my-foo . args)
  (apply foo (cons (Concept "PLN") args)))

(my-foo #:optional-arg 2)
