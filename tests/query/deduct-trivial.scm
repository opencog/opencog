;;
;; deduct-trivial.scm
;;
;; Trivial example of deduction.
;;

(define (stv mean conf) (cog-new-stv mean conf))

;; The Englishman lives in the red house.
(EvaluationLink (stv 1 1)
	(PredicateNode "Nationality")
	(ListLink
		(AvatarNode "person1")
		(ConceptNode "British")
	)
)

(EvaluationLink (stv 1 1)
	(PredicateNode "LivesIn")
	(ListLink
		(AvatarNode "person1")
		(ConceptNode "red_house")
	)
)

;; The person who lives in the red house keeps fish.
(EvaluationLink (stv 1 1)
	(PredicateNode "LivesIn")
	(ListLink
		(AvatarNode "person2")
		(ConceptNode "red_house")
	)
)

(EvaluationLink (stv 1 1)
	(PredicateNode "KeepsPet")
	(ListLink
		(AvatarNode "person2")
		(ConceptNode "fish")
	)
)



