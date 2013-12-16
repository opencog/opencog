
;;
;; deduct-keep.scm
;;
;; Print out who keeps what

(define (print-ownership)
	(BindLink
		;; variable declarations
		(ListLink
			(TypedVariableLink
				(VariableNode "$person")
				(VariableTypeNode "AvatarNode")
			)
			(VariableNode "$nationality")
			(VariableNode "$house")
			(VariableNode "$pet")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				(EvaluationLink
					(PredicateNode "Nationality")
					(ListLink
						(VariableNode "$person")
						(VariableNode "$nationality")
					)
				)
				(EvaluationLink
					(PredicateNode "LivesIn")
					(ListLink
						(VariableNode "$person")
						(VariableNode "$house")
					)
				)
				(EvaluationLink
					(PredicateNode "KeepsPet")
					(ListLink
						(VariableNode "$person")
						(VariableNode "$pet")
					)
				)
			)
			(OrderedLink
				(VariableNode "$person")
				(VariableNode "$nationality")
				(VariableNode "$house")
				(VariableNode "$pet")
			)
		)
	)
)


