(define rule1
	(BindLink
		(VariableList
			(VariableNode "$x")
		)
		(AndLink
			(InheritanceLink
				(VariableNode "$x")
				(ConceptNode "croaks")
			)
			(EvaluationLink
				(PredicateNode "eats")
				(ListLink
					(VariableNode "$x")
					(ConceptNode "flies")
				)
			)
		)
		(InheritanceLink
			(VariableNode "$x")
			(ConceptNode "frog")
		)
	)
)

(define rule2
	(BindLink
		(VariableNode "$x")
		(InheritanceLink
			(VariableNode "$x")
			(ConceptNode "frog")
		)
		(InheritanceLink
			(VariableNode "$x")
			(ConceptNode "green")
		)
	)
)


(define source1
	(InheritanceLink
		(ConceptNode "fritz")
		(ConceptNode "croaks")
	)
)

(EvaluationLink
	(PredicateNode "eats")
	(ListLink
		(ConceptNode "fritz")
		(ConceptNode "flies")
	)
)

;-------------------------------------------
(define wiki (ConceptNode "wikipedia-fc"))


(InheritanceLink  ; Defining a rule base
	(ConceptNode "wikipedia-fc")
	(ConceptNode "URE")
)


(ExecutionLink
   (SchemaNode "URE:maximum-iterations")
   (ConceptNode "wikipedia-fc")
   (NumberNode 20)
)

(MemberLink (stv 0.9 1)
	rule1
	(ConceptNode "wikipedia-fc")
)

(MemberLink (stv 0.5 1)
	rule2
	(ConceptNode "wikipedia-fc")
)
