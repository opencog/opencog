;
; rules.scm
;

; -----------------------------------------------------------------
(define (x)
	; Sentence: "Lisbon is the capital of Portugaul"
	; The RelEx parse is:
	;   _subj(be, Lisbon)
	;   _obj(be, capital)
	;   of(capital, Portugaul)
	;
	; We expect the following variable grounding:
	; var0=Lisbon, var1=capital var2=Portugaul
	;
	(r-varscope
		(r-and
			; We are looking for sentences anchored to the 
			; triples-processing node.
			(r-anchor-trips "$sent")
		
			; $var0 and $var1 are word-instances that must 
			; belong to the same sentence.
			(r-decl-word-inst "$var0" "$sent") 
			(r-decl-word-inst "$var1" "$sent")
		
			; Match subject and object as indicated above.
			(r-rlx "_subj" "be" "$var0")
			(r-rlx "_obj" "be" "$var1")
	
			; Match the proposition
			(r-rlx "$prep" "$var1" "$var2")

			; Get the lemma form of the word instance
			(r-decl-lemma "$var1" "$word1")

			; Convert to a phrase
			(r-rlx "$phrase" "$word1" "$prep")
		)
		; The implicand
		(r-rlx "$phrase" "$var2" "$var0")
	)
)


; ------------------------ END OF FILE ----------------------------
; -----------------------------------------------------------------
