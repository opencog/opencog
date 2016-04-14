(use-modules (ice-9 pretty-print))
(define (mapConceptualizeString input)
   (ListLink 
    (map (lambda (x) (ConceptNode x )) (string-split input #\ )))
   )

; (mapConceptualizeString "I love you")

(define (genQueryPattern input)
   (PatternLink
        (BindLink
           (mapConceptualizeString input)
           (VariableNode "$impl2")
        )))

;; the difference is we already have a scheme list of concepts as input
(define (genSRAIPattern input)
 	   (PatternLink
        (BindLink
           (ListLink input)
           (VariableNode "$impl2")
        )))
		
(BindLink
   (ListLink
      (ConceptNode "I")
      (VariableNode "$star")
      (ConceptNode "you"))
   (ListLink
      (ConceptNode "I")
      (VariableNode "$star")
      (ConceptNode "you")
      (ConceptNode "too")))

(BindLink
   (ListLink
      (ConceptNode "I")
      (ConceptNode "love")
      (VariableNode "$star"))
   (ListLink
      (ConceptNode "I")
      (ConceptNode "like")
      (VariableNode "$star")
      (ConceptNode "a")
      (ConceptNode "lot!")))


(define (findQueryPatterns input)
     (cog-recognize (genQueryPattern input)))

(define (findQuerySRAIPatterns input)
     (cog-recognize (genSRAIPattern input)))
	 
(define (generateReply input)
  (map cog-bind (cog-outgoing-set (findQueryPatterns input))))

(define (generateSRAIReply input)
  (map cog-bind (cog-outgoing-set (findQuerySRAIPatterns input))))
  
  
(define atomspace-stack '())
(define (push-atomspace)
	(set! atomspace-stack (cons (cog-atomspace) atomspace-stack))
	(cog-set-atomspace! (cog-new-atomspace (cog-atomspace))))

(define (pop-atomspace)
	(if (not (null-list? atomspace-stack))
		(begin
			(cog-set-atomspace! (car atomspace-stack))
			(set! atomspace-stack (cdr atomspace-stack))
			(gc) (gc) (gc)) ; MUST GC OR ELSE DELTE ATOMSPACE STICKS AROUND!!
		(throw 'badpop "More pops than pushes!")))


; (push-atomspace)
; (generateReply "I love you")
; (pop-atomspace)
; (push-atomspace)
; (generateReply "I want you")
; (pop-atomspace)
; (push-atomspace)
; (generateReply "I need you")
; (pop-atomspace)


; (cog-prt-atomspace)
; (cog-incoming-set (ConceptNode "love"))
; (cog-incoming-set (ConceptNode "too"))

; cog-atomspace-uuid
; cog-atomspace-clear
; cog-as

; (cog-atomspace-uuid (cog-as (car (cog-incoming-set (ConceptNode "love")))))
; shows 16 .. why is it not deleted ? Oh, its nnot garbage colected!

; (cog-atomspace-uuid (cog-as (car (cog-incoming-set (ConceptNode "too")))))

(define (answerInput input)
 (push-atomspace)
 (pretty-print (generateReply input))
 (cog-extract-recursive (mapConceptualizeString input))
 (pop-atomspace)
)

(define (sraiAnswer input)
 (let ( (ans (list)) (ansList (list)) )
  (begin
	(set! ans (generateSRAIReply input))
	(if (pair? ans)
	    (begin
			(set! ansList (cog-get-all-nodes (car ans)))
			(if (equal? (cog-name (car ansList)) "AIMLSRAI")
				(begin
					;;(display "ans1:")(pretty-print ans)(newline)
					(set! ans (sraiAnswer (cdr ansList)))
					;;(display "ans2:")(pretty-print ans)(newline)
				)
			)
		)
	)
	ans
   )
  )
)

(define (answerInput2 input)
 (let ( (ans (list)) (ansList (list)) )
  (begin
	(push-atomspace)

	(set! ans (generateReply input))

	(set! ansList (cog-get-all-nodes (car ans)))
	(if (equal? (cog-name (car ansList)) "AIMLSRAI")
	    (begin
		    ;;(display "ans1:")(pretty-print ans)(newline)
			(set! ans (sraiAnswer (cdr ansList)))
			;;(display "ans2:")(pretty-print ans)(newline)
		)
	)
	 (pop-atomspace)
	;;(display "ans:")(pretty-print ans)(newline)
	;;(display "ansList:")(pretty-print ansList)(newline)
	ans
   )
 )
)





;;=================================
;; command line test cases
;;=================================
;
;(answerInput "WHAT IS MELODRAMA")
;(generateReply "WHO INVENTED RADIO")
;(answerInput "WHO INVENTED RADIO")
;(answerInput "HOW MANY LITERS ARE IN A GALLON")
;(answerInput "I see you")
;(answerInput "A DOG IS A MAMMAL")
;(answerInput "A DOLPHIN IS A SMART MAMMAL")
;(answerInput "WHO INVENTED PAPER")
;
;(BindLink
;  (ListLink
;    (ConceptNode "WHO")
;    (ConceptNode "INVENTED")
;    (ConceptNode "RADIO"))
;  (ListLink
;    (ConceptNode "Marconi")
;    (ConceptNode "developed")
;    (ConceptNode "the")
;    (ConceptNode "first")
;    (ConceptNode "practical")
;    (ConceptNode "wireless.")))
;
