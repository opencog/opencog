;;
;; deduct-einstein.scm
;;
;; Full set of Einstein Puzzle facts.
;; There are 15 explicitly stated facts, and four additional implicit
;; facts about ordinal counting (ordering of houses in a row).
;;
;; The facts are stated in a fashion that is as close as possible to
;; the natural-language source. The point being that we want the
;; expression of facts to be closely tied to human patterns of speech.
;; Being "efficient" or "clever" is NOT the point.

(define (stv mean conf) (cog-new-stv mean conf))

;; A little handly-dandy utility to avoid over-reporting of "obvious"
;; results. We declare that person1 is the same as person1, etc.
;; A kind-of pauli-exclusion-principle at work.
(define (same person)
	(EvaluationLink (stv 1 1)
		(PredicateNode "IsSamePerson")
		(ListLink
			(FeatureNode person) ; AvatarNode
			(FeatureNode person) ; AvatarNode
		)
	)
)

;; A declaration of fact: it is true that pred has value for person.
(define (fact person pred value)
	(same person)
	(EvaluationLink (stv 1 1)
		(PredicateNode pred)
		(ListLink
			(FeatureNode person)
			(ConceptNode value)
		)
	)
)

;; A neighbor-predicate: two people live next to each other.
(define (neighbor person1 person2)
	(same person1)
	(same person2)
	(EvaluationLink (stv 1 1)
		(PredicateNode "Neighbor")
		(ListLink
			(FeatureNode person1)
			(FeatureNode person2)
		)
	)
)

;; A left-of predicate: one house is left of another
(define (left-of house1 house2)
	(EvaluationLink (stv 1 1)
		(PredicateNode "LeftOf")
		(ListLink
			(ConceptNode house1)
			(ConceptNode house2)
		)
	)
)


;; 1. The Englishman lives in the red house.
(fact "person1" "Nationality" "Englishman")
(fact "person1" "LivesIn" "red house")

;; 2. The Swede keeps dogs.
(fact "person2" "Nationality" "Swede")
(fact "person2" "Keeps" "dogs")

;; 3. The Dane drinks tea.
(fact "person3" "Nationality" "Dane")
(fact "person3" "Drinks" "tea")

;; 4. The green house is just to the left of the white one.
(left-of "green house" "white house")

;; 5. The owner of the green house drinks coffee.
(fact "person5" "LivesIn" "green house")
(fact "person5" "Drinks" "coffee")

;; 6. The Pall Mall smoker keeps birds.
(fact "person6" "Smokes" "PallMall")
(fact "person6" "Keeps" "birds")

;; 7. The owner of the yellow house smokes Dunhills.
(fact "person7" "Smokes" "Dunhill")
(fact "person7" "LivesIn" "yellow house")

;; 8. The man in the center house drinks milk.
(fact "person8" "Drinks" "milk")
(fact "person8" "Address" "103 Main Street")

;; 9. The Norwegian lives in the first house.
(fact "person9" "Nationality" "Norwegian")
(fact "person9" "Address" "101 Main Street")

;; 10. The Blend smoker has a neighbor who keeps cats.
(fact "person10" "Smokes" "Blend")
(neighbor "person10" "catperson")
(fact "catperson" "Keeps" "cats")

;; 11. The man who smokes Blue Masters drinks bier.
(fact "person11" "Smokes" "Blue Master")
(fact "person11" "Drinks" "bier")

;; 12. The man who keeps horses lives next to the Dunhill smoker.
(fact "person12" "Keeps" "horses")
(neighbor "person12" "dun_smoke_person")
(fact "dun_smoke_person" "Smokes" "Dunhill")

;; 13. The German smokes Prince.
(fact "person13" "Nationality" "German")
(fact "person13" "Smokes" "Prince")

;; 14. The Norwegian lives next to the blue house.
(fact "person14" "Nationality" "Norwegian")
(neighbor "person14" "blue_person")
(fact "blue_person" "LivesIn" "blue house")

;; 15. The Blend smoker has a neighbor who drinks water.
(fact "person15" "Smokes" "Blend")
(neighbor "person15" "water_person")
(fact "water_person" "Drinks" "water")

;; ---------------------------------------------------------------
;; Assorted supplemental facts.  These are somehow implicit in the
;; problem statement. We'd mostly like to derive these, from more
;; basic assumptions, but, for now, we'l just state them.
;;
;; A supplemental fact for fact 4: someone lives in the white house.
(fact "person4" "LivesIn" "white house")

;; Supplemental fact: someone keeps fish.
(fact "fish_person" "Keeps" "fish")

;; State some implicitly assumed facts about neighboring houses
;; This is the 'successor' function for ordinal numbers.
(define (successor house1 house2)
	(EvaluationLink (stv 1 1)
		(PredicateNode "Successor")
		(ListLink
			(ConceptNode house1)
			(ConceptNode house2)
		)
	)
)

(successor "101 Main Street" "102 Main Street")
(successor "102 Main Street" "103 Main Street")
(successor "103 Main Street" "104 Main Street")
(successor "104 Main Street" "105 Main Street")

;; ---------------------------------------------------------------
;; By-process-of-elimination facts
;; If person doesn't live in one of the four houses, they must live in
;; the fifth. Likewsie, if person doesn't smoke/drink/keep one of the four,
;; they must have the fifth.

(define (is-a x y)
	(InheritanceLink
		(ConceptNode x)
		(ConceptNode y)
	)
)

(is-a "red house" "house")
(is-a "white house" "house")
(is-a "green house" "house")
(is-a "yellow house" "house")
(is-a "blue house" "house")

(is-a "water" "drink")
(is-a "milk" "drink")
(is-a "bier" "drink")
(is-a "coffee" "drink")
(is-a "tea" "drink")

(is-a "Prince" "tobacco")
(is-a "PallMall" "tobacco")
(is-a "Dunhill" "tobacco")
(is-a "Blend" "tobacco")
(is-a "Blue Master" "tobacco")

(is-a "fish" "pet")
(is-a "dogs" "pet")
(is-a "birds" "pet")
(is-a "cats" "pet")
(is-a "horses" "pet")

(is-a "Englishman" "citizenship")
(is-a "Swede" "citizenship")
(is-a "Dane" "citizenship")
(is-a "Norwegian" "citizenship")
(is-a "German" "citizenship")
