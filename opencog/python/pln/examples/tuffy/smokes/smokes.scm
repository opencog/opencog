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

;; Rules

;; If X smokes, then X has cancer.
;; ForAll(x) Smokes(x) -> Cancer(x)
; MLN Rule Weight: 0.5
; Approximate probability: 0.6225

; Version #1
(ImplicationLink (stv 0.6225 1.0)
    (EvaluationLink
        smokes
        (ListLink
            (VariableNode "$X")))
    (EvaluationLink
        cancer
        (ListLink
            (VariableNode "$X"))))

;; In the case that X and Y are friends, if X smokes then so does Y.
;; ForAll(x,y) Friends(x,y) -> (Smokes(x) <-> Smokes(y))
; MLN Rule Weight: 0.4
; Approximate probability: 0.5987

; Version #3
(ImplicationLink (stv 0.5987 1.0)
    (EvaluationLink
        friends
        (ListLink
            (VariableNode "$X")
            (VariableNode "$Y")))
    (ImplicationLink
        (EvaluationLink
            smokes
            (ListLink
                (VariableNode "$X")))
        (EvaluationLink
            smokes
            (ListLink
                (VariableNode "$Y")))))

; If X and Y are friends, then Y and X are friends.
; Note: this is not currently used.
(EquivalenceLink (stv 1.0 1.0)
    (EvaluationLink
        friends
        (ListLink
            (VariableNode "$X")
            (VariableNode "$Y")))
    (EvaluationLink
        friends
        (ListLink
            (VariableNode "$Y")
            (VariableNode "$X"))))

;;; --------------------------------------------------------------------------
;;; evidence.db

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
