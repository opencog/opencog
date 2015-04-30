;
; Simple question answering
;
; plan of attack:
; implement (cog-member? which takes MemberLink, subsitutes
; (calls sustitute method, which is already implemented) and
; thenasks atom-table directly about it
;
;;; Assert a basic fact
;;;  |- likes(Tom, baseball) 
(EvaluationLink
	(PredicateNode "likes")
	(ListLink
		(ConceptNode "Tom")
		(ConceptNode "baseball")
	)
)

;;; A satisfiability query: Does Tom like $X?
;;; The EvaluationLink just asserts that "Tom does like X" (as a fact).
;;; The SatisfactionLink turns it into a question: the SatisfactionLink
;;; can evaluate to true or false, depending on what X is.
;;; Note that the SatisfactionLink is in the form of a lambda; that is,
;;; it has the form  Lx.(Tom likes X)
;;;
(SatisfactionLink
	(VariableNode "$X")
	(EvaluationLink
		(PredicateNode "likes")
		(ListLink
			(ConceptNode "Tom")
			(VariableNode "$X")
		)
	)
)

;;; A satisfiability question: Does Tom like X where X is baseball?
(MemberLink
	(ConceptNode "baseball")
	(SatisfactionLink
		(VariableNode "$X")
		(EvaluationLink
			(PredicateNode "likes")
			(ListLink
				(ConceptNode "Tom")
				(VariableNode "$X")
			)
		)
	)
)
