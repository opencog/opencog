;;
;; deduct-trivial.scm
;;
;; Trivial example of deduction.
;;
;; Part of the "Einstein puzzle" demo.
;;

(define (stv mean conf) (cog-new-stv mean conf))

;; The Englishman lives in the red house.
(EvaluationLink (stv 1 1)
	(PredicateNode "Nationality")
	(ListLink
		(FeatureNode "person1") ; AvatarNode
		(ConceptNode "British")
	)
)

(EvaluationLink (stv 1 1)
	(PredicateNode "LivesIn")
	(ListLink
		(FeatureNode "person1") ; AvatarNode
		(ConceptNode "red_house")
	)
)

;; The person who lives in the red house keeps fish.
(EvaluationLink (stv 1 1)
	(PredicateNode "LivesIn")
	(ListLink
		(FeatureNode "person2")
		(ConceptNode "red_house")
	)
)

(EvaluationLink (stv 1 1)
	(PredicateNode "KeepsPet")
	(ListLink
		(FeatureNode "person2")
		(ConceptNode "fish")
	)
)



