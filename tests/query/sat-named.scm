;
; Simple named question answering
;
; Implmentation: need to look for definition corresponding to the second
; element of the member link, and then it is a simple atomspace check
;
;;; Assert basic fact
;;;  |- likes(Tom, baseball) 
(EvaluationLink
	(PredicateNode "likes")
	(ListLink
		(ConceptNode "Tom")
		(ConceptNode "baseball")
	)
)

;;;
;;; A named satisfiability query: Does Tom like $X?
;;; The EvaluationLink just asserts that "Tom does like X" (as a fact).
;;; The SatisfactionLink turns it into a question: the SatisfactionLink
;;; can evaluate to true or false, depending on what X is.
;;; Note that the SatisfactionLink is in the form of a lambda; that is,
;;; it has the form  Lx.(Tom likes X)
;;;
;;; Finally, we want to give the above a name, so that we can refer to it
;;; in other places. We use the DefineLink to do this. Given a lambda,
;;; for example, Lx.(stuff) which is just an anonymous function taking x,
;;; the DefineLink turns it into a named function: so that 
;;;    Define name Lx.(stuff)
;;; is the same as 
;;;    (name x).(stuff)
;;;
;;; So, below, the DefineLink merely gives the question a name: It just
;;; says that there is a particular question, which is given the name 
;;; "Does Tom like X?", and that this question takes a single variable
;;; i.e $X.  We infer this variable from the SatisfactionLink.  That is,
;;; the DefineLink binds exactly the same variables that the lambda under
;;; it does (with SatisfactionLink being the lambda).
(DefineLink
	(ConceptNode "Does Tom like X?")
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

;;; A satisfiability question: Does Tom like X where X is baseball?
(MemberLink
	(ConceptNode "baseball")
	(ConceptNode "Does Tom like X?")
)
