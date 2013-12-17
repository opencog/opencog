;;
;; deduct-einstein.scm
;;
;; Full set of Einstein Puzzle facts
;;

(define (stv mean conf) (cog-new-stv mean conf))

;; A little handly-dandy utility to avoid over-reporting of "obvious"
;; results. We declare that person1 is the same asm person1, etc.
;; A kind-of pauli-exclusion-principle at work.
(define (same person)
	(EvaluationLink (stv 1 1)
		(PredicateNode "IsSamePerson")
		(ListLink
			(AvatarNode person)
			(AvatarNode person)
		)
	)
)

;; A declaration of fact: it is true that pred has value for person.
(define (fact person pred value)
	(same person)
	(EvaluationLink (stv 1 1)
		(PredicateNode pred)
		(ListLink
			(AvatarNode person)
			(ConceptNode value)
		)
	)
)

;; A neighbor-predicate
(define (neighbor person1 person2)
	(same person1)
	(same person2)
	(EvaluationLink (stv 1 1)
		(PredicateNode "Neighbor")
		(ListLink
			(AvatarNode person1)
			(AvatarNode person2)
		)
	)
)


;; 1. The Englishman lives in the red house.
(fact "person1" "Nationality" "British")
(fact "person1" "LivesIn" "red house")

;; 2. The Swede keeps dogs.
(fact "person2" "Nationality" "Swede")
(fact "person2" "Keeps" "dogs")

;; 3. The Dane drinks tea.
(fact "person3" "Nationality" "Dane")
(fact "person3" "Drinks" "tea")

;; 4. The green house is just to the left of the white one.
;; ()

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
;; ()

;; 15. The Blend smoker has a neighbor who drinks water.
(fact "person15" "Smokes" "Blend")
(neighbor "person15" "water_person")
(fact "water_person" "Drinks" "water")

;; State some facts about neighboring houses
(define (left-of house1 house2)
	(EvaluationLink (stv 1 1)
		(PredicateNode "LeftOf")
		(ListLink
			(ConceptNode house1)
			(ConceptNode house2)
		)
	)
)

(left-of "101 Main Street" "102 Main Street")
(left-of "102 Main Street" "103 Main Street")
(left-of "103 Main Street" "104 Main Street")
(left-of "104 Main Street" "105 Main Street")

