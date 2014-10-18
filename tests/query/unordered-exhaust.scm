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
				

