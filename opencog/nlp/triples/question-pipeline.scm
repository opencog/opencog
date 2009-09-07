;
; question-pipeline.scm
; 
; An experimental set of question-answering rules
;
; Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
;
; -----------------------------------------------------------------
; The following answers a simple WH-question (who, what, when etc.)
; The question is presumed to be a simple triple itself.
;
; To limit the scope of the search (for performance), the bottom
; of the prep-triple is assumed to be anchored.
;
; Basically, we are trying to handle triples of the form
; "capital_of(France, what)" and verifying that "what" is a query,
; and then yanking out the answer.
; 
; # IF %ListLink("# TRIPLE BOTTOM ANCHOR", $qvar) 
;       ^ $tripl($word-inst, $qvar)     ; the question
;       ^ &query_var($qvar)             ; validate WH-question
;       ^ %LemmaLink($word-inst, $word) ; common word-instance
;       ^ %LemmaLink($join-inst, $word) ; common seme
;       ^ $tripl($join-inst, $ans)      ; answer
;       ^ ! &query_var($ans)            ; answer should NOT be a query
;    THEN
;       ^3_&declare_answer($ans)
 
(define (wh-question wh-clause ans-clause)
	(r-varscope
		(r-and
			(r-anchor-node *bottom-anchor* "$qvar")

			wh-clause  ; the prep-phrase we are matching!

			;; XXX someday, this needs to be an or-list of WH- words.
			(r-rlx-flag "what" "$qvar")
			(r-decl-lemma  "$word-inst" "$word")
			(r-decl-lemma  "$join-inst" "$word")

			ans-clause

			(r-not (r-rlx-flag "what" "$ans"))
		)
		(r-anchor-node *query-soln-anchor* "$ans")
	)
)

(define question-rule-0
	(wh-question 
		(r-rlx "$tripl" "$word-inst" "$qvar")
		(r-rlx "$tripl" "$join-inst" "$ans")
	)
)

(define question-rule-1
	(wh-question 
		(r-rlx "$tripl" "$qvar" "$word-inst")
		(r-rlx "$tripl" "$ans"  "$join-inst")
	)
)

; -----------------------------------------------------------------
; 0
; Truth-query question: "Did John throw a rock?"
;        _subj(throw, John)
;        _obj(throw, rock)
;        HYP (throw, T)
;        TRUTH-QUERY-FLAG (throw, T)
;
; or more generally "did X verb Y?"

; Find the seme for this word-instance
(define (r-seme-of-word-inst word-inst seme)
	(r-and 
		(r-link InheritanceLink word-inst seme)
		(r-decl-vartype "WordInstanceNode" word-inst)
		(r-decl-vartype "SemeNode" seme)
	)
)

(define truth-query-rule-0
	(r-varscope
		(r-and
			(r-anchor "# NEW PARSES" "$sent")
			(r-decl-word-inst "$verb" "$sent")

			; Identify the question.
			(r-rlx "_subj" "$verb" "$svar")
			(r-rlx "_obj"  "$verb" "$ovar")
			(r-rlx-flag "hyp" "$verb")
			(r-rlx-flag "truth-query" "$verb")

			; abstract to words (XXX - should be semes!!)
			(r-seme-of-word-inst "$svar" "$ans-svar")
			(r-seme-of-word-inst "$ovar" "$ans-ovar")
			(r-seme-of-word-inst "$verb" "$ans-verb")
			
			; Now look for a matching assertion with these semes
			(r-rlx "_subj" "$ans-verb" "$ans-svar")
			(r-rlx "_obj"  "$ans-verb" "$ans-ovar")

			; the belwo are probably not needed -- for semes.
			(r-not (r-rlx-flag "hyp" "$ans-verb"))
			(r-not (r-rlx-flag "truth-query" "$ans-verb"))
		)
		(r-anchor-node *query-soln-anchor* "$ans-verb")
	)
)

; -----------------------------------------------------------------
;
(define *question-rule-list* (list
	question-rule-0
	question-rule-1
))

(define *truth-query-rule-list* (list
	truth-query-rule-0
))

; ------------------------ END OF FILE ----------------------------
; -----------------------------------------------------------------

