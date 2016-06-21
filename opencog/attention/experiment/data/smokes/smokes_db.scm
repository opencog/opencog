;;;; smokes.scm
;;;; PLN representation of the "smokes" sample from Tuffy Markov Logic Networks

;;; More details on this sample are available here:
;;; https://github.com/cosmoharrigan/tuffy/tree/master/samples/smoke
;;; http://hazy.cs.wisc.edu/hazy/tuffy/doc/tuffy-manual.pdf

;;; --------------------------------------------------------------------------
;;; prog.mln

;; Evidence and query predicates and concepts:

(define friends (PredicateNode "friends"))
(define smokes (PredicateNode "smokes"))
(define cancer (PredicateNode "cancer"))

(define Anna (ConceptNode "Anna" (stv 0.1667 1)))
(define Bob (ConceptNode "Bob" (stv 0.1667 1)))
(define Edward (ConceptNode "Edward" (stv 0.1667 1)))
(define Frank (ConceptNode "Frank" (stv 0.1667 1)))
(define Gary (ConceptNode "Gary" (stv 0.1667 1)))
(define Helen (ConceptNode "Helen" (stv 0.1667 1)))

;;; --------------------------------------------------------------------------
;;; evidence.db

;(EvaluationLink (stv 0.4 1.0) (PredicateNode "friends") (ListLink (ConceptNode (stv 1.0 1.0) "RandFriend-d172ed2a")(ConceptNode (stv 1.0 1.0) "RandFriend-8845c5d3")))
;(EvaluationLink (stv 0.4 1.0) (PredicateNode "friends") (ListLink (ConceptNode (stv 1.0 1.0) "RandFriend-73e25b6c")(ConceptNode (stv 1.0 1.0) "RandFriend-3e059c94")))

; Anna and Bob are friends.
(EvaluationLink (stv 1.0 1.0)
    friends
    (ListLink
        Anna
        Bob))

(EvaluationLink (stv 1.0 1.0)
    friends
    (ListLink
        Anna
        Edward))

(EvaluationLink (stv 1.0 1.0)
    friends
    (ListLink
        Anna
        Frank))

(EvaluationLink (stv 1.0 1.0)
    friends
    (ListLink
        Edward
        Frank))

(EvaluationLink (stv 1.0 1.0)
    friends
    (ListLink
        Gary
        Helen))

;; Note: the 'non-friendships' are not explicitly defined in this version

; Anna smokes.
(EvaluationLink (stv 1.0 1.0)
    smokes
    (ListLink
        Anna))

(EvaluationLink (stv 1.0 1.0)
    smokes
    (ListLink
        Edward))


; Extra data
;-------------------------------------------------

(define neighbors (PredicateNode "neighbors"))
(define knows (PredicateNode "knows"))
(define hasMet (PredicateNode "HasMet"))
(define sick (PredicateNode "sick"))
(define tired (PredicateNode "tired"))
(define intelligent (ConceptNode "intelligent"))
(define successful (ConceptNode "successful"))
(define happy (ConceptNode "happy"))

; Anna Bob Edward Frank Gary Helen
; Cancer: Bob, Frank, Anna, Edward

; A and B are neighbors.
(EvaluationLink (stv 1.0 1.0)
    neighbors
    (ListLink
        Anna
        Frank))

(EvaluationLink (stv 1.0 1.0)
    neighbors
    (ListLink
        Edward
        Bob))

(EvaluationLink (stv 1.0 1.0)
    neighbors
    (ListLink
        Edward
        Gary))

(EvaluationLink (stv 1.0 1.0)
    neighbors
    (ListLink
        Gary
        Helen))

; Neighbors know each other.
(ImplicationLink (stv 0.9 1.0)
    (EvaluationLink
        neighbors
        (ListLink
            (VariableNode "$X")
            (VariableNode "$Y")))
    (EvaluationLink
        knows
        (ListLink
            (VariableNode "$X")
            (VariableNode "$Y"))))

; People who know each other have met.
(ImplicationLink (stv 0.9 1.0)
    (EvaluationLink
        knows
        (ListLink
            (VariableNode "$X")
            (VariableNode "$Y")))
    (EvaluationLink
        hasMet
        (ListLink
            (VariableNode "$X")
            (VariableNode "$Y"))))

; A is intelligent.
(InheritanceLink (stv 0.9 1.0)
    Anna
    intelligent)

(InheritanceLink (stv 0.9 1.0)
    Frank
    intelligent)

(InheritanceLink (stv 0.9 1.0)
    Gary
    intelligent)

(InheritanceLink (stv 0.9 1.0)
    Bob
    intelligent)

(InheritanceLink (stv 0.9 1.0)
    Helen
    intelligent)

; Intelligent people are successful.
(ImplicationLink (stv 0.9 1.0)
    (InheritanceLink
        (VariableNode "$X")
        intelligent)
    (InheritanceLink
        (VariableNode "$X")
        successful))

; Successful people are happy.
(ImplicationLink (stv 0.9 1.0)
    (InheritanceLink
        (VariableNode "$X")
        successful)
    (InheritanceLink
        (VariableNode "$X")
        happy))

; People who have cancer are sick.
(ImplicationLink (stv 1.0 1.0)
    (EvaluationLink
        cancer
        (ListLink
            (VariableNode "$X")))
    (EvaluationLink
        sick
        (ListLink
            (VariableNode "$X"))))

; People who are sick are tired.
(ImplicationLink (stv 1.0 1.0)
    (EvaluationLink
        sick
        (ListLink
            (VariableNode "$X")))
    (EvaluationLink
        tired
        (ListLink
            (VariableNode "$X"))))
;;; --------------------------------------------------------------------------
;;; query.db

; Who has cancer?
(define hasCancer
    (EvaluationLink
        cancer
        (ListLink
            (VariableNode "$hasCancer"))))



;;; --------------------------------------------------------------------------
;;; Attention allocation configuration

(cog-set-av! cancer (av 1000 0 0))
(cog-set-af-boundary! 1)
