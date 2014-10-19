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

;; This should match in 4! + 4! = 48 different ways, with 4! for
;; the xyzw permuations, and another 4! for the pqr-set permuations.
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
;; explosion.  The embedded SetLink match is unique
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

