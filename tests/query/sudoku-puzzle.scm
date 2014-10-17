;
; Definition for a specific puzzle
;

(define (puzzle)
	(BindLink
		(variable-decls)
		(ImplicationLink
			(AndLink
				(sudoku-constraints)
			)
			; The solution
			(variable-decls)
		)
	)
)

(define (solver pzl) (cog-bind pzl))
