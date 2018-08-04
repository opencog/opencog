; Copyright (C) 2016 OpenCog Foundation

;; --------------------------------------------------------------
;; Helper function
(define (init-state)
"
  Used for initializing
"
    (StateLink (faces-node) (NumberNode 0))
)

(define (inc-faces)
"
  Increments the number of faces perceived by 1
"
    (define (num-of-faces)
        (string->number (cog-name (car
            (cog-chase-link 'StateLink 'NumberNode (faces-node))))))

    (StateLink
        (ConceptNode "Total number of faces around")
        (NumberNode (+ (num-of-faces) 1)))
)

(define (test)
    (equal? (cog-tv (sociality-behavior (faces-node))) (cog-tv (faces-node)))
)
