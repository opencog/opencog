;
; prep-rules.scm
;
; An experimental set of copula/preposition-mangling rules. 
;
; See "rule-tools.scm" for more information about the file format.
; See "prep-maps.scm" for "make-prep-phrase"
;
; Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
;
; -----------------------------------------------------------
; The following rules create word-preposition linkages; these must
; be created in advance, before the later rules run. These do not
; need to be precise (its OK to generate some garbage, I think(??)
; but its important that all prep phrases be considered.
;
(define (prep-rule-template rel a b)
	(r-varscope
		(r-and
			(r-anchor-trips "$sent")
			(r-decl-word-inst "$var1" "$sent")
			(r-rlx rel a b)
			(r-rlx "$prep" "$var1" "$var2")
			(r-decl-vartype "PrepositionalRelationshipNode" "$prep")
			(r-decl-lemma "$var1" "$word1")
		)
		(r-schema "scm:make-prep-phrase" "$word1" "$prep")
	)
)

; -----------------------------------------------------------
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var1,$sent)  ; limit scope of rule
;       ^ _obj(be,$var1)
;       ^ $prep($var1,$var2)              ; preposition
;       ^ %LemmaLink($var1,$word1)        ; word of word instance
;       THEN ^3_&scm:make-prep-phrase($word1,$prep)

(define prep-rule-0
	(prep-rule-template "_obj" "be" "$var1")
)

; -----------------------------------------------------------
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var1,$sent)  ; limit scope of rule
;       ^ _subj(be,$var1)
;       ^ $prep($var1,$var2)              ; preposition
;       ^ %LemmaLink($var1,$word1)        ; word of word instance
;       THEN ^3_&scm:make-prep-phrase($word1,$prep)

(define prep-rule-1
	(prep-rule-template "_subj" "be" "$var1")
)

; -----------------------------------------------------------
; ; Sentence: "The color of the sky is blue."
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var1,$sent)  ; limit scope of rule
;       ^ _predadj($var1,$var0)
;       ^ $prep($var1,$var2)              ; preposition 
;       ^ %LemmaLink($var1,$word1)        ; word of word instance
;       THEN ^3_&scm:make-prep-phrase($word1,$prep)
 
(define prep-rule-2
	(prep-rule-template "_predadj" "$var1" "$var0")
)

; -----------------------------------------------------------
; ; Sentence: "Pottery is made from clay."
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var1,$sent)  ; limit scope of rule
;       ^ _obj($var1,$var0)
;       ^ $prep($var1,$var2)              ; preposition 
;       ^ %LemmaLink($var1,$word1)        ; word of word instance
;       THEN ^3_&scm:make-prep-phrase($word1,$prep)
 
(define prep-rule-3
	(prep-rule-template "_obj" "$var1" "$var0")
)

; -----------------------------------------------------------
; Sentence: "The cat sat on the mat"
;
(define prep-rule-4
	(prep-rule-template "_subj" "$var1" "$var0")
)

; -----------------------------------------------------------
; Sentence: What did the cat sit on? 
; This treats "on" as a particle, and creates a polyword "sit_on"
;
(define prep-rule-5
	(r-varscope
		(r-and
			(r-anchor-trips "$sent")
			(r-decl-word-inst "$var1" "$sent")
			(r-rlx "_subj" "$pwi" "$var1")
			(r-rlx-flag "polyword" "$pwi")
			(r-decl-lemma "$pwi" "$pw")
		)
		(r-schema "scm:make-polyword-phrase" "$pw")
	)
)

; -----------------------------------------------------------
; The list below is needed by the triples-processing pipeline
;
(define prep-rule-list (list
   prep-rule-0
   prep-rule-1
   prep-rule-2
   prep-rule-3
   prep-rule-4
   prep-rule-5
))

; ----------------------- END OF FILE -----------------------
