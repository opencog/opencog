

(define special_wn   (list
                      (WordNode "manager") 
                      (WordNode "champion")
                      (WordNode "Olympian")
                      (WordNode "Athleth")
                      (WordNode "Messi")
                      (WordNode "Ronaldo")                      
                      )
  )

;Test
(ListLink (WordNode "manager") (WordNode "Olympian"))
(ListLink (WordNode "Athleth") (WordNode "Ronaldo"))

(define (contains? lst elem)
    (if (empty? lst) #f
            (or (eq? (car lst) elem) (contains? (cdr lst) elem))))

(define (isSwHeb listt)
    (if (and (member (car listt) special_wn) (member (cadr listt) special_wn))
        #t
        #f
    )
)

(define (swHebLinks hlist)  
    (define SwHebList '())
    (map (lambda (elem)
        (if (isSwHeb (cog-outgoing-set elem))
            (set! SwHebList (append SwHebList (list elem))) 
        ))
      hlist)

    SwHebList
)

;Test
(define getSwListLinks (swHebLinks (cog-get-atoms 'ListLink) ))

;Get all HebbianLinks created between special Word nodes.
(define getSwHebLinks (swHebLinks (cog-get-atoms 'AsymmetricHebbianLink) ))


