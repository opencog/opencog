(define eval_a (EvaluationLink (PredicateNode "A") (stv 0.5 0.8)))
(define eval_b (EvaluationLink (PredicateNode "B")))
(define imp_eva_evb (ImplicationLink eval_a eval_b (stv 0.85 0.79)))

(define eval_c (EvaluationLink (PredicateNode "C")))
(define imp_evb_evc (ImplicationLink eval_b eval_c (stv 0.85 0.79)))

(define eval_d (EvaluationLink (PredicateNode "D")))
(define imp_evc_evd (ImplicationLink eval_c eval_d (stv 0.85 0.79)))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		(InheritanceLink human mortal (stv 0.5 1))
		(InheritanceLink socrates human (stv 0.5 1))
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "ModusPonensRule<ImplicationLink>")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "10")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		(EvaluationLink (PredicateNode "D"))
	)
)

(EvaluationLink (PredicateNode "query") (ListLink eval_d))
(EvaluationLink (PredicateNode "rules") (ListLink (ConceptNode "ModusPonensRule<ImplicationLink>")))

