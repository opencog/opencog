;
; Definition for a specific puzzle
; Hand-typed-in version of 
; http://www.theguardian.com/lifeandstyle/2014/oct/17/sudoku-2944-hard
;
; XXX As of 18 October 2014, the pattern matcher fails to find a
; solution.  This is probably due to a bug in the pattern matcher,
; although it might be due to a bug in the coding below ...
; XXXXXXXXXXXXXXXXX

; Certain fixed numbers appear in certain fixed cell locations.
(EvaluationLink (PredicateNode "fix12") (ConceptNode "eight"))
(EvaluationLink (PredicateNode "fix15") (ConceptNode "seven"))
(EvaluationLink (PredicateNode "fix18") (ConceptNode "six"))

(EvaluationLink (PredicateNode "fix21") (ConceptNode "one"))
(EvaluationLink (PredicateNode "fix24") (ConceptNode "six"))
(EvaluationLink (PredicateNode "fix26") (ConceptNode "eight"))
(EvaluationLink (PredicateNode "fix29") (ConceptNode "seven"))

(EvaluationLink (PredicateNode "fix33") (ConceptNode "five"))
(EvaluationLink (PredicateNode "fix37") (ConceptNode "one"))

(EvaluationLink (PredicateNode "fix42") (ConceptNode "three"))
(EvaluationLink (PredicateNode "fix48") (ConceptNode "four"))

(EvaluationLink (PredicateNode "fix51") (ConceptNode "four"))
(EvaluationLink (PredicateNode "fix59") (ConceptNode "nine"))

(EvaluationLink (PredicateNode "fix62") (ConceptNode "nine"))
(EvaluationLink (PredicateNode "fix68") (ConceptNode "five"))

(EvaluationLink (PredicateNode "fix73") (ConceptNode "four"))
(EvaluationLink (PredicateNode "fix77") (ConceptNode "six"))

(EvaluationLink (PredicateNode "fix81") (ConceptNode "nine"))
(EvaluationLink (PredicateNode "fix84") (ConceptNode "five"))
(EvaluationLink (PredicateNode "fix86") (ConceptNode "four"))
(EvaluationLink (PredicateNode "fix89") (ConceptNode "one"))

(EvaluationLink (PredicateNode "fix92") (ConceptNode "seven"))
(EvaluationLink (PredicateNode "fix95") (ConceptNode "three"))
(EvaluationLink (PredicateNode "fix98") (ConceptNode "eight"))

(define (puzzle)
	(BindLink
		; There are eighty-one variables! 81 = 9x9 cells
		(variable-decls)
		(ImplicationLink
			(AndLink
				; For this puzzle, 24 of the variables are fixed immediately.
				(EvaluationLink (PredicateNode "fix12") (VariableNode "$cell_12"))
				(EvaluationLink (PredicateNode "fix15") (VariableNode "$cell_15"))
				(EvaluationLink (PredicateNode "fix18") (VariableNode "$cell_18"))

				(EvaluationLink (PredicateNode "fix21") (VariableNode "$cell_21"))
				(EvaluationLink (PredicateNode "fix24") (VariableNode "$cell_24"))
				(EvaluationLink (PredicateNode "fix26") (VariableNode "$cell_26"))
				(EvaluationLink (PredicateNode "fix29") (VariableNode "$cell_29"))

				(EvaluationLink (PredicateNode "fix33") (VariableNode "$cell_33"))
				(EvaluationLink (PredicateNode "fix37") (VariableNode "$cell_37"))

				(EvaluationLink (PredicateNode "fix42") (VariableNode "$cell_42"))
				(EvaluationLink (PredicateNode "fix48") (VariableNode "$cell_48"))

				(EvaluationLink (PredicateNode "fix51") (VariableNode "$cell_51"))
				(EvaluationLink (PredicateNode "fix59") (VariableNode "$cell_59"))

				(EvaluationLink (PredicateNode "fix62") (VariableNode "$cell_62"))
				(EvaluationLink (PredicateNode "fix68") (VariableNode "$cell_68"))

				(EvaluationLink (PredicateNode "fix73") (VariableNode "$cell_73"))
				(EvaluationLink (PredicateNode "fix77") (VariableNode "$cell_77"))

				(EvaluationLink (PredicateNode "fix81") (VariableNode "$cell_81"))
				(EvaluationLink (PredicateNode "fix84") (VariableNode "$cell_84"))
				(EvaluationLink (PredicateNode "fix86") (VariableNode "$cell_86"))
				(EvaluationLink (PredicateNode "fix89") (VariableNode "$cell_89"))

				(EvaluationLink (PredicateNode "fix92") (VariableNode "$cell_92"))
				(EvaluationLink (PredicateNode "fix95") (VariableNode "$cell_95"))
				(EvaluationLink (PredicateNode "fix98") (VariableNode "$cell_98"))

				; Aside from the above 24 constraints, there are another 
				; there are 81+27 constraints. 81 of these say that each
				; of the 81 variables must be a number.  The remaining 27
				; constraints state that nine columns, rows and boxes must
				; have the right form.
				(sudoku-constraints)
			)
			; The solution
			(variable-decls)
		)
	)
)
