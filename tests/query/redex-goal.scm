;
; Simple goal solving using redex's
;
(use-modules (opencog))
(use-modules (opencog query))

;;; Assert basic fact
;;;  |- likes(Tom, baseball) 
(EvaluationLink
	(PredicateNode "likes")
	(ListLink
		(ConceptNode "Tom")
		(ConceptNode "baseball")
	)
)

;;; Assert implication
;;;   |- likes(Tom,$X) -> likes(Bill, $X) 
(ImplicationLink
	(EvaluationLink
		(PredicateNode "likes")
		(ListLink
			(ConceptNode "Tom")
			(VariableNode "$X")
		)
	)
	(EvaluationLink
		(PredicateNode "likes")
		(ListLink
			(ConceptNode "Bill")
			(VariableNode "$X")
		)
	)
)

;;; Question to be answered: is it true that likes(Bill, baseball)?
;;; i.e. can we show that |- likes(Bill, baseball)

(BetaRedex
	(ConceptNode "liking-rule")
	(ListLink
		(ConceptNode "baseball")
	)
)

;;;
;;; A named satisfiability query: Does Bill like $X?
;;; The EvaluationLink just asserts that "Bill does like X" (as a fact).
;;; The SatisfactionLink turns it into a question: the SatisfactionLink
;;; can evaluate to true or false, depending on what X is.
;;; Note that the SatisfactionLink is in the form of a lambda; that is,
;;; it has the form  Lx.(Bill likes X)
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
;;; "Does Bill like X?", and that this question takes a single variable
;;; i.e $X.  We infer this variable from the SatisfactionLink.  That is,
;;; the DefineLink binds exactly the same variables that the lambda under
;;; it does (with SatisfactionLink being the lambda).
(DefineLink
	(ConceptNode "Does Bill like X?")
	(SatisfactionLink
		(VariableNode "$X")
		(EvaluationLink
			(PredicateNode "likes")
			(ListLink
				(ConceptNode "Bill")
				(VariableNode "$X")
			)
		)
	)
)

;;; A satisfiability question: Does Bill like X where X is baseball?
(MemberLink
	(ConceptNode "baseball")
	(ConceptNode "Does Bill like X?")
)

;; solution:
;; do plain member link, get false,
;; look for  sat link body as second half of implication
;; pattern match first half of implication, if found
;; try to check member again.

