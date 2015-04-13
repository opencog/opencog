;
; Test the exhaustive search of the unordered link.
; We expect all possible permuations to be traversed.
;

; The raw data: a set containing four objects.
(SetLink
	(ConceptNode "x")
	(ConceptNode "y")
	(ConceptNode "z")
	(ConceptNode "w")
)

; Another set containing four objects.
(SetLink
	(ConceptNode "p")
	(ConceptNode "q")
	(ConceptNode "r")
	(SetLink
		(ConceptNode "s")
		(ConceptNode "t")
		(ConceptNode "u")
	)
)

; Yet another set containing four objects.
(SetLink
	(ConceptNode "a")
	(ConceptNode "b")
	(ConceptNode "c")
	(SetLink
		(ConceptNode "a")
		(ConceptNode "b")
		(ConceptNode "c")
	)
)

;; This should match in 4! + (6+6+6) + (6+6+6) = 60 different ways,
;; with 4! for the xyzw permuations, and (6+6+6)=18 for the pqr-set
;; permuations, and another (6+6+6)=18 for the abc-set.
(define (exhaust)
   (BindLink
      ;; variable decls
      (ListLink
         (TypedVariableLink
            (VariableNode "$a")
            (VariableTypeNode "ConceptNode"))
         (VariableNode "$b")
         (VariableNode "$c")
         (VariableNode "$d")
      )
		(ImplicationLink
			(AndLink
				(SetLink ; sets are inherenetly unordered
         		(VariableNode "$a")
         		(VariableNode "$b")
         		(VariableNode "$c")
         		(VariableNode "$d")
				)
			)
			; The result to report
			(ListLink
        		(VariableNode "$a")
        		(VariableNode "$b")
        		(VariableNode "$c")
        		(VariableNode "$d")
			)
		)
	)
)

;; This should match in 3! * 3! = 36 different ways, viz a combinatorial
;; explosion.  The embedded SetLink match is unique (which is why its not
;; 4! as there's no possible ambiguity there).
;;
;; Acutally there should be 36+36=72 solutions, 36 for the pqr-set and
;; another 36 for the abc-set.
;;
;; The goal of this test is to check nested unordered links: viz one
;; unordered link inside another, so that proper state presevation and
;; backtracking is needed to correctly handle the nesting.
(define (exhaust-2)
   (BindLink
      ;; variable decls
      (ListLink
         (TypedVariableLink
            (VariableNode "$a")
            (VariableTypeNode "ConceptNode"))
         (VariableNode "$b")
         (VariableNode "$c")
         (VariableNode "$d")
         (VariableNode "$e")
         (VariableNode "$f")
      )
		(ImplicationLink
			(AndLink
				(SetLink ; sets are inherenetly unordered
         		(VariableNode "$a")
         		(VariableNode "$b")
         		(VariableNode "$c")
					(SetLink
         			(VariableNode "$d")
         			(VariableNode "$e")
         			(VariableNode "$f")
					)
				)
			)
			; The result to report
			(ListLink
        		(VariableNode "$a")
        		(VariableNode "$b")
        		(VariableNode "$c")
        		(VariableNode "$d")
        		(VariableNode "$e")
        		(VariableNode "$f")
			)
		)
	)
)

;; This should match in (3! * 3!) / 3 = 12 different ways, viz a 
;; constrained combinatorial explosion.  That is, since $a $b $c
;; can have 3! assignments, and $c $d $e can have 3! assignments,
;; but the first and the second $c must be equal, thus the cosets
;; are modulo-3 according to this equality constraint.
;;
;; The goal of this test is to check nested unordered links: viz one
;; unordered link inside another, so that proper state presevation and
;; backtracking is needed to correctly handle the nesting.
(define (exhaust-3)
   (BindLink
      ;; variable decls
      (ListLink
         (TypedVariableLink
            (VariableNode "$a")
            (VariableTypeNode "ConceptNode"))
         (VariableNode "$b")
         (VariableNode "$c")
         (VariableNode "$d")
         (VariableNode "$e")
      )
		(ImplicationLink
			(AndLink
				(SetLink ; sets are inherenetly unordered
         		(VariableNode "$a")
         		(VariableNode "$b")
         		(VariableNode "$c")
					(SetLink
         			(VariableNode "$c")
         			(VariableNode "$d")
         			(VariableNode "$e")
					)
				)
			)
			; The result to report
			(ListLink
        		(VariableNode "$a")
        		(VariableNode "$b")
        		(VariableNode "$c")
        		(VariableNode "$d")
        		(VariableNode "$e")
			)
		)
	)
)

;; This should match in (3! * 3!) / 6 = 6 different ways, viz a 
;; constrained combinatorial explosion.  That is, since $a $b $c
;; can have 3! assignments, and $b $c $d can have 3! assignments,
;; but the first and the second $b must be equal, thus the cosets
;; are modulo-3 according to this equality constraint. Then, the
;; first and second $c must also be equal, giving the modulo-2.
;;
;; The goal of this test is to check nested unordered links: viz one
;; unordered link inside another, so that proper state presevation and
;; backtracking is needed to correctly handle the nesting.
(define (exhaust-4)
   (BindLink
      ;; variable decls
      (ListLink
         (TypedVariableLink
            (VariableNode "$a")
            (VariableTypeNode "ConceptNode"))
         (VariableNode "$b")
         (VariableNode "$c")
         (VariableNode "$d")
      )
		(ImplicationLink
			(AndLink
				(SetLink ; sets are inherenetly unordered
         		(VariableNode "$a")
         		(VariableNode "$b")
         		(VariableNode "$c")
					(SetLink
         			(VariableNode "$b")
         			(VariableNode "$c")
         			(VariableNode "$d")
					)
				)
			)
			; The result to report
			(ListLink
        		(VariableNode "$a")
        		(VariableNode "$b")
        		(VariableNode "$c")
        		(VariableNode "$d")
			)
		)
	)
)

;; This should match in (3! * 3!) / 6 = 6 different ways, viz a 
;; constrained combinatorial explosion.  That is, since the first 
;; $a $b $c can have 3! assignments, and the other $a $b $c can have
;; 3! assignments, but the order of the first and second must be equal.
;; As above, there is a modulo-3 and a modulo-2 division, and also a
;; modulo-1 that changes nothing.
;;
;; The goal of this test is to check nested unordered links: viz one
;; unordered link inside another, so that proper state presevation and
;; backtracking is needed to correctly handle the nesting.
(define (exhaust-5)
   (BindLink
      ;; variable decls
      (ListLink
         (TypedVariableLink
            (VariableNode "$a")
            (VariableTypeNode "ConceptNode"))
         (VariableNode "$b")
         (VariableNode "$c")
      )
		(ImplicationLink
			(AndLink
				(SetLink ; sets are inherenetly unordered
         		(VariableNode "$a")
         		(VariableNode "$b")
         		(VariableNode "$c")
					(SetLink
         			(VariableNode "$a")
         			(VariableNode "$b")
         			(VariableNode "$c")
					)
				)
			)
			; The result to report
			(ListLink
        		(VariableNode "$a")
        		(VariableNode "$b")
        		(VariableNode "$c")
			)
		)
	)
)

;; This should match in (3! * 3!) / 3 = 12 different ways, viz a 
;; constrained combinatorial explosion.  That is, since $a $b $c
;; can have 3! assignments, and $c $d $e can have 3! assignments,
;; but the first and the second $c must be equal, thus the cosets
;; are modulo-3 according to this equality constraint.
;;
;; The goal of this test is to check nested unordered links: viz one
;; unordered link inside another, so that proper state presevation and
;; backtracking is needed to correctly handle the nesting.

(EvaluationLink
	(PredicateNode "equal")
	(ListLink
		(ConceptNode "a")
		(ConceptNode "a")
	)
)

(EvaluationLink
	(PredicateNode "equal")
	(ListLink
		(ConceptNode "b")
		(ConceptNode "b")
	)
)

(EvaluationLink
	(PredicateNode "equal")
	(ListLink
		(ConceptNode "c")
		(ConceptNode "c")
	)
)

(define (exhaust-eq-12)
   (BindLink
      ;; variable decls
      (ListLink
         (TypedVariableLink
            (VariableNode "$a")
            (VariableTypeNode "ConceptNode"))
         (VariableNode "$b")
         (VariableNode "$c1")
         (VariableNode "$c2")
         (VariableNode "$e")
         (VariableNode "$f")
      )
		(ImplicationLink
			(AndLink
				(SetLink ; sets are inherenetly unordered
         		(VariableNode "$a")
         		(VariableNode "$b")
         		(VariableNode "$c1")
					(SetLink
         			(VariableNode "$c2")
         			(VariableNode "$e")
         			(VariableNode "$f")
					)
				)

				; External clause enforcing equality relation
				(EvaluationLink
					(PredicateNode "equal")
					(ListLink
						(VariableNode "$c1")
						(VariableNode "$c2")
					)
				)
			)
			; The result to report
			(ListLink
        		(VariableNode "$a")
        		(VariableNode "$b")
        		(VariableNode "$c1")
        		(VariableNode "$c2")
        		(VariableNode "$e")
        		(VariableNode "$f")
			)
		)
	)
)

(define (exhaust-eq-6)
   (BindLink
      ;; variable decls
      (ListLink
         (TypedVariableLink
            (VariableNode "$a")
            (VariableTypeNode "ConceptNode"))
         (VariableNode "$b1")
         (VariableNode "$b2")
         (VariableNode "$c1")
         (VariableNode "$c2")
         (VariableNode "$f")
      )
		(ImplicationLink
			(AndLink
				(SetLink ; sets are inherenetly unordered
         		(VariableNode "$a")
         		(VariableNode "$b1")
         		(VariableNode "$c1")
					(SetLink
         			(VariableNode "$c2")
         			(VariableNode "$b2")
         			(VariableNode "$f")
					)
				)

				; External clause enforcing equality relation
				(EvaluationLink
					(PredicateNode "equal")
					(ListLink
						(VariableNode "$b1")
						(VariableNode "$b2")
					)
				)
				(EvaluationLink
					(PredicateNode "equal")
					(ListLink
						(VariableNode "$c1")
						(VariableNode "$c2")
					)
				)
			)
			; The result to report
			(ListLink
        		(VariableNode "$a")
        		(VariableNode "$b1")
        		(VariableNode "$b2")
        		(VariableNode "$c1")
        		(VariableNode "$c2")
        		(VariableNode "$f")
			)
		)
	)
)

