;
; rules.scm
;
; An experimental set of copula/preposition-mangling rules. 
;
; These rules generate triples holding word instances.
; At a later stage, after we decide that we "like" a given triple,
; i.ethat it appropriately manifests some semantic relation, then
; it is "canonicalized" by replacing the word instances by the
; appropriate SemeNodes represneting the underlying conncept.
;
; Some quick notes on evaluation: conjuncts are evaluated from 
; first to last, and so the order of the terms matters. Terms that
; narrow down the search the most dramatically should come first,
; so as to avoid an overly-broad search of the atomspace.
;
; All of these rules are structured so that a search is performed
; only over sentences That are tagged with a link to the node
; "# APPLY TRIPLE RULES". Since this rule is first, this prevents
; a search over the entire atomspace.
;
; Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
;
; -----------------------------------------------------------
; 0
; Sentence: "Lisbon is the capital of Portugaul"
; var0=Lisbon, var1=capital var2=Portugaul
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var0,$sent)  ; $var0 and $var1 must be
;       ^ %WordInstanceLink($var1,$sent)  ; in the same sentence
;       ^ _subj(be,$var0) ^ _obj(be,$var1)
;       ^ $prep($var1,$var2)              ; preposition 
;       ^ %LemmaLink($var1,$word1)        ; word of word instance
;       ^ $phrase($word1, $prep)          ; convert to phrase
;       THEN ^3_$phrase($var2, $var0) 

(define triple-rule-0

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
		
			; The word "be" must occure in the sentence
			(r-decl-word-inst "$be" "$sent")
			(r-decl-lemma "$be" "be")

			; Match subject and object as indicated above.
			(r-rlx "_subj" "$be" "$var0")
			(r-rlx "_obj" "$be" "$var1")
	
			; Match the proposition
			(r-decl-word-inst "$var2" "$sent")
			(r-rlx "$prep" "$var1" "$var2")
			(r-decl-vartype "PrepositionalRelationshipNode" "$prep")

			; Get the lemma form of the word instance
			(r-decl-lemma "$var1" "$word1")

			; Convert to a phrase
			(r-rlx "$phrase" "$word1" "$prep")
		)
		; The implicand
		(r-rlx "$phrase" "$var2" "$var0")
	)
)

; -----------------------------------------------------------------
; 1
; Sentence: "The capital of Germany is Berlin"
; var0=capital, var1=Berlin var2=Germany
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var0,$sent)  ; $var0 and $var1 must be
;       ^ %WordInstanceLink($var1,$sent)  ; in the same sentence
;       ^ _subj(be,$var0) ^ _obj(be,$var1)   ; reversed subj, obj
;       ^ $prep($var0,$var2)              ; preposition 
;       ^ %LemmaLink($var0,$word0)        ; word of word instance
;       ^ $phrase($word0, $prep)          ; convert to phrase
;       THEN ^3_$phrase($var2, $var1) 
; 
; Note that the above rule is "reversed", in that, if the prep was
; dropped, it loooks like a backwards IsA statement.

(define triple-rule-1
	(r-varscope
		(r-and
			(r-anchor-trips "$sent")
			(r-decl-word-inst "$be" "$sent")
			(r-decl-lemma "$be" "be")
			(r-rlx "_subj" "$be" "$var0")
			(r-rlx "_obj" "$be" "$var1")
			(r-decl-word-inst "$var2" "$sent")
			(r-rlx "$prep" "$var0" "$var2")
			(r-decl-vartype "PrepositionalRelationshipNode" "$prep")
			(r-decl-lemma "$var0" "$word0")
			(r-rlx "$phrase" "$word0" "$prep")
		)
		(r-rlx "$phrase" "$var2" "$var1")
	)
)

; -----------------------------------------------------------------
; 2
; Sentence: "The color of the sky is blue."
; var0=blue, var1=color, var2=sky  $prep=of
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var1,$sent)  ; scope to sentence
;       ^ _predadj($var1,$var0)
;       ^ $prep($var1,$var2)              ; preposition 
;       ^ %LemmaLink($var1,$word1)        ; word of word instance
;       ^ $phrase($word1, $prep)          ; convert to phrase
;       THEN ^3_$phrase($var2, $var0) 

(define triple-rule-2
	(r-varscope
		(r-and
			(r-anchor-trips "$sent")
			(r-decl-word-inst "$var1" "$sent")
			(r-rlx "_predadj" "$var1" "$var0")
			(r-rlx "$prep" "$var1" "$var2")
			(r-decl-vartype "PrepositionalRelationshipNode" "$prep")
			(r-decl-lemma "$var1" "$word1")
			(r-rlx "$phrase" "$word1" "$prep")
		)
		(r-rlx "$phrase" "$var2" "$var0")
	)
)

; -----------------------------------------------------------------
; 3
; Sentence: "Pottery is made from clay."
; Parse:
;    _obj(make, pottery)
;    from(make, clay)
;
; var0=make  var1=pottery var2=clay  prep=from
;
; However, we want to reject a match to [Madrid is a city in Spain.]
; which has a similar pattern -- but it has a subj, while the pottery
; example does not. See also rule-8 for a similar caase, with missing
; obj.
;
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var0,$sent)  ; scope to sentence
;       ^ !_subj($var0,$var-unwanted)     ; Must not have a subj
;       ^ _obj($var0,$var1)
;       ^ $prep($var0,$var2)              ; preposition 
;       ^ %LemmaLink($var0,$word0)        ; word of word instance
;       ^ $phrase($word0, $prep)          ; convert to phrase
;       THEN ^3_$phrase($var2, $var1) 

(define triple-rule-3
	(r-varscope
		(r-and
			(r-anchor-trips "$sent")
			(r-decl-word-inst "$var0" "$sent")
			(r-not (r-rlx "_subj" "$var0" "$var-unwanted"))
			(r-rlx "_obj" "$var0" "$var1")
			(r-rlx "$prep" "$var0" "$var2")
			(r-decl-vartype "PrepositionalRelationshipNode" "$prep")
			(r-decl-lemma "$var0" "$word0")
			(r-rlx "$phrase" "$word0" "$prep")
		)
		(r-rlx "$phrase" "$var2" "$var1")
	)
)

; -----------------------------------------------------------------
; 4
; Sentence: "Yarn is made of fibers."
; var1=yarn  var2=fibers  phinst=made_of
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($phinst,$sent)  ; scope to sentence
;       ^ _obj($phinst,$var1) ^ _iobj($phinst,$var2)
;       ^ %LemmaLink($phinst,$phrase)     ; word of word instance
;       THEN ^3_$phrase($var2, $var1) 

(define triple-rule-4
	(r-varscope
		(r-and
			(r-anchor-trips "$sent")
			(r-decl-word-inst "$phinst" "$sent")
			(r-rlx "_obj"  "$phinst" "$var1")
			(r-rlx "_iobj" "$phinst" "$var2")
			(r-decl-lemma "$phinst" "$phrase")
		)
		(r-rlx "$phrase" "$var2" "$var1")
	)
)

; -----------------------------------------------------------------
; 5
; Sentence: "The heart is in the chest." 
; also, "The garage is behind the house"
; var1=heart  var2=chest phinst=in
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($phinst,$sent)  ; scope to sentence
;       ^ _psubj($phinst,$var1) ^ _pobj($phinst,$var2)
;       ^ $prep(_%copula, $phinst)
;       ^ %LemmaLink($phinst, $prword)
;       ^ %ListLink($prep,$prword)
;       THEN ^3_$prep($var2, $var1) 

(define triple-rule-5
	(r-varscope
		(r-and
			(r-anchor-trips "$sent")
			(r-decl-word-inst "$phinst" "$sent")
			(r-rlx "_psubj" "$phinst" "$var1")
			(r-rlx "_pobj"  "$phinst" "$var2")
			(r-decl-lemma "$phinst" "$prword")
			(r-decl-prep "$prep" "$prword")
		)
		(r-rlx "$prep" "$var2" "$var1")
	)
)

; -----------------------------------------------------------------
; 6
; Sentence "Berlin is a city"
; var1=Berlin var2=city
; Must be careful not to make the pattern too simple, as, for example,
; "the captial of Germany is Berlin", the prep is crucial to reversing
; the order of the subject and object! Alternately, demand that $var2
; is indefinite.
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var1,$sent)  ; $var1 and $var2 must be
;       ^ %WordInstanceLink($var2,$sent)  ; in the same sentence
;       ^ %WordInstanceLink(be,$sent)     ; hyp-flag "be" in same sentence
;       ^ _subj(be, $var1) ^ _obj(be, $var2)
;       ^ !DEFINITE-FLAG($var2)           ; accept "a city" but not "the city"
;       ^ !HYP-FLAG(be)                   ; Disallow "Is Berlin a city?"
;       THEN ^3_isa($var2, $var1) 

(define triple-rule-6
	(r-varscope
		(r-and
			(r-anchor-trips "$sent")
			(r-decl-word-inst "$be" "$sent")
			(r-decl-lemma "$be" "be")
			(r-rlx "_subj" "$be" "$var1")
			(r-rlx "_obj"  "$be" "$var2")
			(r-not (r-rlx-flag "hyp" "$be"))
			(r-not (r-rlx-flag "definite" "$var2"))
		)
		(r-rlx "isa" "$var2" "$var1")
	)
)

; -----------------------------------------------------------------
; 7
; Truth-query question: "Is Berlin a city?"
; var1=Berlin var2=city
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var1,$sent)  ; $var1 and $var2 must be
;       ^ %WordInstanceLink($var2,$sent)  ; in the same sentence
;       ^ %WordInstanceLink(be,$sent)     ; hyp-flag "be" in same sentence
;       ^ _subj(be, $var1) ^ _obj(be, $var2)
;       ^ HYP-FLAG(be)                    ; Hyp must be present.
;       THEN ^3_hypothetical_isa($var2, $var1) 

(define triple-rule-7
	(r-varscope
		(r-and
			(r-anchor-trips "$sent")
			(r-decl-word-inst "$be" "$sent")
			(r-decl-lemma "$be" "be")
			(r-rlx "_subj" "$be" "$var1")
			(r-rlx "_obj"  "$be" "$var2")
			(r-rlx-flag "hyp" "$be")
		)
		(r-rlx "hypothetical_isa" "$var2" "$var1")
	)
)

; -----------------------------------------------------------------
; 8
; Sentence "The cat sat on the mat"
; Parse:
;        _subj(sit, cat)
;        on(sit, mat)
; Desired output:  sit_on(mat,cat)
; Note missing _obj in sentence -- need to reject sentences with objects.
; This is very similar to rule-3 above.
; This also picks up alternate parses for [Paris is in France] and 
; [The heart is in the chest] with var0==be.
;
(define triple-rule-8
	(r-varscope
		(r-and
			(r-anchor-trips "$sent")
			(r-decl-word-inst "$var0" "$sent")
			(r-rlx "_subj" "$var0" "$var1")
			(r-not (r-rlx "_obj" "$var0" "$var-unwanted"))
			(r-rlx "$prep" "$var0" "$var2")
			(r-decl-vartype "PrepositionalRelationshipNode" "$prep")
			(r-decl-lemma "$var0" "$word0")
			(r-rlx "$phrase" "$word0" "$prep")
		)
		(r-rlx "$phrase" "$var2" "$var1")
	)
)

; -----------------------------------------------------------------
; 9
; Sentence "What did the cat sit on?"
; This has two parses; one with a normal preposition construction, 
; and one which treats "on" as a particle, so:
;
;    _subj(sit_on, cat)
;    _obj(sit_on, _$qVar)
;    POLYWORD-FLAG(sit_on, T)
;    QUERY-TYPE(_$qVar, what)
;
; This rule presumes that such polyword constructions are suiteable
; triples (which might not always be the case?)

(define triple-rule-9
	(r-varscope
		(r-and
			(r-anchor-trips "$sent")
			(r-decl-word-inst "$var0" "$sent")
			(r-rlx "_subj" "$var0" "$var1")
			(r-rlx "_obj" "$var0" "$var2")
			(r-rlx-flag "polyword" "$var0") ; check for polyword flag
			; wtf ... perhaps below should be $var0 and not $polyword? ??
			; (r-decl-vartype "PrepositionalRelationshipNode" "$polyword")
			(r-decl-lemma "$var0" "$pw")
			(r-rlx "$pwrel" "$pw" "$pwrel") ; promote to define relation
		)
		(r-rlx "$pwrel" "$var2" "$var1")
	)
)

; -----------------------------------------------------------------
; Sentence "Men are mortal"
; Has a *single* depedency:
;     _predadj(man, mortal)
;
; var1=mortal var2=man
; Must reject prepositions, so that e.g. "the color (of the sky) is blue." 
; which contains _predadj as well as a prep "of", is rejected.
; XXX This rejects sentences with prep phrases, but it should also probably
; reject anything with _subj, _obj, etc. XXX
;
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var2, $sent)  ; scope to sentence
;       ^ _predadj($var2, $var1) 
;       ^ ! $prep($var2,$var3)
;       THEN ^3_isa($var1, $var2)
; 
(define triple-rule-10
	(r-varscope
		(r-and
			(r-anchor-trips "$sent")
			(r-decl-word-inst "$var2" "$sent")
			(r-rlx "_predadj" "$var2" "$var1")

			; Must NOT have a prep phrase in it!
			(r-not (r-rlx "$prep" "$var2" "$var3"))
			(r-decl-vartype "PrepositionalRelationshipNode" "$prep")
			(r-decl-vartype "WordInstanceNode" "$var3")
		)
		(r-rlx "isa" "$var1" "$var2")
	)
)

; -----------------------------------------------------------------
; Sentence "The color (of the sky) is blue."
; if the prep is present, then reverse the order.
; var1=blue var2=color
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var2,$sent)  ; scope to sentence
;       ^ _predadj($var2, $var1) 
;       ^ $prep($var2,$var3)
;       ^ !HYP-FLAG($var1)       ; Disallow "Is Socrates mortal?"
;       THEN ^3_isa($var2, $var1)
; 
; Sentence: "Berlin is in Germany", alternate parse
; (This becomes the dominant parse for "Berlin is a city in Germany").
; var1=Berlin var2=Germany $prep=in
; However, this can't work if defined as simply as this: there are
; too many bogus matches for 'prep'.
; XXX FIXME (this is same, similar problem to the other XXX above.)
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;      ^ %WordInstanceLink($var1,$sent)  ; $var1 and $var2 must be
;      ^ %WordInstanceLink($var2,$sent)  ; in the same sentence
;      ^ _subj(be, $var1) ^ $prep(be, $var2)
;      THEN ^3_$prep($var2, $var1) 
; 
; 
; Question: What are bats made of?
; var0=bat $vrb=make $prep=of $qvar=what
; XXX for some reason, this isn't working, not sure why ... 
; this needs debugging.
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var0,$sent)  ; $var0 and $vrb must be
;       ^ %WordInstanceLink($qvarinst,$sent) ; in the same sentence
;       ^ _subj(be,$var0)
;       ^ _obj($vrb,$var0)
;       ^ _obj($prepinst,$qvarinst)
;       ^ $prep(be, $prepinst)            ; preposition 
;       ^ %LemmaLink($vrb,$word1)         ; word of word instance
;       ^ $phrase($word1, $prep)          ; convert to phrase
;       ^ %LemmaLink($qvarintst,$qvar)    ; word of word instance
;       THEN ^3_$phrase($qvar, $var0) 


; -----------------------------------------------------------------
; -----------------------------------------------------------------
; -----------------------------------------------------------------
; 1000
; Rules to identify truth assertions. We need to identify thruth assersions
; so that seme-promotion works correctly: we need to seme-promote the subject,
; object for these kinds of sentences, but do not want to do seme promotions
; on triple-style assertions (these are handled above).
;
; Truth assertion: "John threw a rock"
;        _subj(throw, John)
;        _obj(throw, rock)
; 
; or more generally "X verbed Y".  Use this to identify sentences that do *not* 
; have prepositional statements in them. 
;

(define truth-assertion-rule-0
	(r-varscope
		(r-and
			(r-anchor "# NEW PARSES" "$sent")
			(r-decl-word-inst "$verb" "$sent")

			; Identify the assertion.
			(r-rlx "_subj" "$verb" "$svar")
			(r-rlx "_obj"  "$verb" "$ovar")

			; Must not be a question.
			(r-not (r-rlx-flag "hyp" "$verb"))
			(r-not (r-rlx-flag "truth-query" "$verb"))

			; Must NOT have a prep phrase in it!
			(r-not (r-rlx "$prep" "$ovar" "$var3"))
			(r-decl-vartype "PrepositionalRelationshipNode" "$prep")
			(r-decl-vartype "WordInstanceNode" "$var3")
		)
		(r-anchor-node *truth-assertion-anchor* "$verb")
	)
)

; -----------------------------------------------------------------
; 1001 -- Rules to identify truth assertions. 
;
; Truth assertion: "John made fun of Sarah."
;     _subj(make, John)
;     _obj(make, fun)
;     of(fun, Sarah)
;
; or more generally "X verbed Y prep Z". 
;
; XXX FIXME, rule below is just rule above, but without the prep check.
; This means this rule might break one of the triple-style rules above...

(define truth-assertion-rule-1
	(r-varscope
		(r-and
			(r-anchor "# NEW PARSES" "$sent")
			(r-decl-word-inst "$verb" "$sent")

			; Identify the assertion.
			(r-rlx "_subj" "$verb" "$svar")
			(r-rlx "_obj"  "$verb" "$ovar")
			(r-rlx "$prep" "$ovar" "$pvar")
			(r-decl-vartype "PrepositionalRelationshipNode" "$prep")
			(r-decl-vartype "WordInstanceNode" "$pvar")

			; Must not be a question.
			(r-not (r-rlx-flag "hyp" "$verb"))
			(r-not (r-rlx-flag "truth-query" "$verb"))

		)
		(r-anchor-node *truth-assertion-anchor* "$verb")
	)
)

; -----------------------------------------------------------------
; needed by the triples-processing pipeline
;
(define triple-rule-list (list
	triple-rule-0
	triple-rule-1
	triple-rule-2
	triple-rule-3
	triple-rule-4
	triple-rule-5
	triple-rule-6
	triple-rule-7
	triple-rule-8
	triple-rule-9
	triple-rule-10
))

(define truth-assertion-list (list
	truth-assertion-rule-0
	truth-assertion-rule-1
))

; ------------------------ END OF FILE ----------------------------
; -----------------------------------------------------------------
