;; Example to prove that Bob's new interest in Pokemon cards is the
;; cause of his new Canadian friendships.

;; Before March 2007, Bob never had any Canadian friends except those
;; who were also friends of his wife.  After March 2007, Bob started
;; acquiring Canadian friends who were not friends of his wife.  In
;; late 2006, Bob started collecting Pokemon cards. Most of the new
;; Canadian friends Bob made between March 2007 and Late 2007 are
;; associated with Pokemon cards In late 2006, Bob started learning
;; French. Most of the new Canadian friends Bob made between March
;; 2007 and Late 2007 are Quebecois.

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Temporal reasoning definitions ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(define HoldAt (PredicateNode "HoldAt"))
(define HoldThroughout (PredicateNode "HoldThroughout"))
(define InitiatedAt (PredicateNode "InitiatedAt"))
(define InitiatedThroughout (PredicateNode "InitiatedThroughout"))
(define TerminatedAt (PredicateNode "TerminatedAt"))
(define TerminatedThroughout (PredicateNode "TerminatedThroughout"))
(define InitialTime (TimeNode "0"))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Temporal reasoning axioms ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; PredictiveImplicationInitiatedLink
;;    T
;;    A
;;    B
;;
;; is equivalent to
;;
;; PredictiveImplicationLink
;;    T
;;    A_initiated
;;    B_initiated
;;
;; where A_initiated is equivalent to
;;
;; AverageLink
;;     t
;;     initiatedAt
;;         t
;;         A

;;;;;;;;;;;
;; times ;;
;;;;;;;;;;;
(define InitTime (TimeNode "InitTime"))
(define End2006 (TimeNode "End2006"))
(define March2007 (TimeNode "March2007"))
(define End2007 (TimeNode "End2007"))

;;;;;;;;;;;;;
;; persons ;;
;;;;;;;;;;;;;
(define Bob (ConceptNode "Bob"))
(define BobWife (ConceptNode "BobWife"))

;;;;;;;;;;;;;;
;; concepts ;;
;;;;;;;;;;;;;;
(define CanadianPeople (ConceptNode "CanadianPeople"))
(define PokemonCards (ConceptNode "PokemonCards"))
(define FrenchLanguage (ConceptNode "FrenchLanguage"))
(define Quebecois (ConceptNode "Quebecois"))

;;;;;;;;;;;;;;;;
;; predicates ;;
;;;;;;;;;;;;;;;;
(define FriendOf (PredicateNode "FriendOf"))
(define Collecting (PredicateNode "Collecting"))
(define Learning (PredicateNode "Learning"))

;;;;;;;;;;;;;;;
;; variables ;;
;;;;;;;;;;;;;;;
(define X (VariableNode "X"))
(define Y (VariableNode "Y"))

;;;;;;;;;;;;;;;;;
;; definitions ;;
;;;;;;;;;;;;;;;;;

;; Canadian Friend of Bob
;;
;; AndLink
;;     EvaluationLink
;;         FriendOf
;;         ListLink
;;             Bob
;;             X
;;     SubsetLink
;;         X
;;         CanadianPeople
(define CanadianFriendBob (AndLink (EvaluationLink FriendOf
                                                   (ListLink Bob
                                                             X))
                                   (SubsetLink X
                                               CanadianPeople)))

;; BobWifeFriend
(define BobWifeFriend (EvaluationLink FriendOf
                                      (ListLink BobWife
                                                X)))

;; Canadian Friend of Bob but Not His Wife
(define CanadianFriendBobNotWife (AndLink CanadianFriendBob
                                          (NotLink BobWifeFriend)))

;; Friends of Bob are not friends of his wife
(define AverageCFBNW (AverageLink (ListLink X)
                                  CanadianFriendBobNotWife))

;; Bob collecting Pokemon cards
(define BobCollectingPokemonCards (EvaluationLink Collecting
                                                  (ListLink Bob
                                                            PokemonCards)))

;; Canadian friends of Bob who are not friends of his wife are
;; associated with Pokemon cards.
(define CFBNWPokemonCards (AverageLink (ListLink X)
                                       (ImplicationLink CanadianFriendBobNotWife
                                                        (InheritanceLink X
                                                                         PokemonCards))))

;; BobLearningFrench
(define BobLearningFrench (EvaluationLink Learning
                                          (ListLink Bob
                                                    FrenchLanguage)))

;; Canadian friends of Bob who are not friends of his wife are
;; Quebecois.
(define CFBNWQuebecois (AverageLink (ListLink X)
                                   (ImplicationLink CanadianFriendBobNotWife
                                                    (SubsetLink X
                                                                Quebecois))))

;;;;;;;;;;;;
;; Axioms ;;
;;;;;;;;;;;;

;; Before March 2007, Bob never had any Canadian friends except those
;; who were also friends of his wife.
(define axiom1 (EvaluationLink (stv 0 0.9)
                               HoldThroughout
                               (ListLink InitialTime
                                         March2007
                                         AverageCFBNW)))

;; After March 2007, Bob started acquiring Canadian friends who were
;; not friends of his wife.
(define axiom2 (EvaluationLink (stv 0.3 0.9)
                               InitiatedThroughout
                               (ListLink March2007
                                         End2007
                                         AverageCFBNW)))

;; In late 2006, Bob started collecting Pokemon cards.
(define axiom3 (EvaluationLink (stv 1 0.9)
                               InitiatedAt
                               (ListLink End2006
                                         BobCollectingPokemonCards)))

;; If X collects Y then X shares associations with Y
(define axiom4 (AverageLink (stv 0.7 0.7)
                            (ImplicationLink (EvaluationLink Collecting
                                                             (ListLink X Y))
                                             (InheritanceLink X Y))))

;; Most of the new Canadian friends Bob made after March 2007 who are
;; not friends of his wife are associated with Pokemon cards.
(define axiom5 (EvaluationLink (stv 0.6 0.8)
                               HoldThroughout
                               (ListLink March2007
                                         End2007
                                         CFBNWPokemonCards)))

;; In late 2006, Bob started learning French.
(define axiom6 (EvaluationLink (stv 1 0.9)
                               InitiatedAt
                               (ListLink End2006
                                         BobLearningFrench)))

;; If X learns Y then X shares associations with Y
(define axiom7 (AverageLink (stv 0.7 0.7)
                            (ImplicationLink (EvaluationLink Learning
                                                             (ListLink X Y))
                                             (InheritanceLink X Y))))

;; Most of the new Canadian friends Bob made after March 2007 who are
;; not friends of his wife are Quebecois.
(define axiom8 (EvaluationLink (stv 0.7 0.8)
                               HoldThroughout
                               (ListLink March2007
                                         End2007
                                         CFBNWQuebecois)))

;; Quebecois are assicated with French Language
(define axiom9 (InheritanceLink Quebecois FrenchLanguage))

;; Bob's Pokemon cards interest is the cause of Bob's new Canadian
;; friendships
;;
;; PredictiveImplicationInitiatedLink <?>
;;     3Months
;;     Year
;;     BobCollectingPokemonCards
;;     AverageCFBNW

;; Let's go backward...

;; Using the definition of PredictiveInitiatedImplicationLink
;;
;; ImplicationLink
;;     BobCollectingPokemonCards_initiated
;;     SequentialAnd
;;         3Months
;;         Year
;;         BobCollectingPokemonCards_initiated
;;         AverageCFBNW_initiated

;; Due to definition of mixed implication
;;
;; OrLink
;;     ExtensionalImplicationLink
;;         BobCollectingPokemonCards_initiated
;;         SequentialAndLink
;;             3Months
;;             Year
;;             BobCollectingPokemonCards_initiated
;;             AverageCFBNW_initiated
;;     IntensionalImplicationLink
;;         BobCollectingPokemonCards__initiated
;;         SequentialAndLink
;;             3Months
;;             Year
;;             BobCollectingPokemonCards_initiated
;;             AverageCFBNW_initiated

;; Using axiom3 and axiom2 and the definition of SequentialAndLink, one
;; can conclude
;;
;; ExtensionalImplicationLink
;;     BobCollectingPokemonCards_initiated
;;     SequentialAndLink
;;         t+3Months
;;         t+Year
;;         BobCollectingPokemonCards_initiated
;;         AverageCFBNW_initiated

;; The confidence of the TV of such implication will not be very high,
;; because such even has occured only once.

;; Done for extensional implication!

;; Now let's take care of intensional implication

;; Using axiom4 and universal intanstiation with X=Bob and Y=Pokemon
;;
;; InheritanceLink
;;     Bob
;;     PokemonCards

;; Using some assumption
;;
;; InheritanceLink
;;     NotLink
;;         Bob
;;     PokemonCards

;; Given that above one can calculate that PokemonCards is a pattern of Bob
;;
;; MemberLink
;;     PokemonCards
;;     PAT(BobCollectingPokemonCards__initiated)

;; Then using axiom5 one infer that
;;
;; MemberLink
;;     PokemonCards
;;     PAT(SequentialAndLink
;;             3Months
;;             Year
;;             BobCollectingPokemonCards_initiated
;;             AverageCFBNW_initiated)

;; which leads to
;;
;; ExtensionalImplication
;;     PAT(BobCollectingPokemonCards__initiated)
;;     PAT(SequentialAndLink
;;             3Months
;;             Year
;;             BobCollectingPokemonCards_initiated
;;             AverageCFBNW_initiated)

;; and therefore

;; IntensionalImplicationLink
;;     BobCollectingPokemonCards__initiated
;;     SequentialAndLink
;;         3Months
;;         Year
;;         BobCollectingPokemonCards_initiated
;;         AverageCFBNW_initiated
