;
; mst-print.scm
;
; Create a human-readable version of an MST parse.
;

(define (print-mst TXT)
"
 Create a human-readable version of an MST parse. TXT should be a
 string.
"
	(for-each (lambda (section)
		(let* ((word-pair (car section))
				(cost (cdr section))
				(wleft (car word-pair))
				(wright (cdr word-pair))
				(seqleft (car wleft))
				(seqright (car wright))
				(str-left (cog-name (cdr wleft)))
				(str-right (cog-name (cdr wright)))
			)
			(format #t "~A . ~A   <-->  ~A . ~A  cost= ~A\n"
				seqleft str-left seqright str-right cost)
		))
		(foo-parse-text TXT)) 
)
