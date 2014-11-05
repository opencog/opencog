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

;; This should match in 4! + 4! + 4! = 72 different ways, with 4! for
;; the xyzw permuations, and another 4! for the pqr-set permuations,
;; and another 4! for the abc-set.
(define (exhaust)
   (BindLink
      ;; variable decls
      (ListLink
         (VariableNode "$a")
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
         (VariableNode "$a")
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
         (VariableNode "$a")
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

