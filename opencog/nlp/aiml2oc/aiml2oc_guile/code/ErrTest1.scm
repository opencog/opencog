(use-modules (ice-9 pretty-print))
(use-modules (opencog atom-types))

(define (nth n l)
  (if (or (> n (length l)) (< n 0))
    (error "Index out of bounds.")
    (if (eq? n 0)
      (car l)
      (nth (- n 1) (cdr l)))))
	  
	  
(define (mapConceptualizeString input)
   (ListLink 
    (map (lambda (x) (ConceptNode x )) (string-split input #\ )))
   )


(define (randomChoice choice-list)
	(nth (random (length choice-list)) choice-list)
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
      (GlobNode "$star")
      (ConceptNode "you"))
   (ListLink
      (ConceptNode "I")
      (GlobNode "$star")
      (ConceptNode "you")
      (ConceptNode "too")))

(BindLink
   (ListLink
      (ConceptNode "I")
      (ConceptNode "love")
      (GlobNode "$star"))
   (ListLink
      (ConceptNode "I")
      (ConceptNode "like")
      (GlobNode "$star")
      (ConceptNode "a")
      (ConceptNode "lot!")))


(define (findQueryPatterns input)
     (cog-recognize (genQueryPattern input)))

(define (findQuerySRAIPatterns input)
     (cog-recognize (genSRAIPattern input)))
	 
(define (generateReply input)
  (display "generateReply:")(pretty-print input)(newline)
  (let ((queryPatterns  (findQueryPatterns  input)) (outset '()) )
   (begin
     (display "generateReply-queryPatterns:")(pretty-print queryPatterns)(newline)
	 (set! outset (cog-outgoing-set queryPatterns))
	 (display "generateReply-outset:")(pretty-print outset)(newline)
     (map cog-bind outset)
   )
  )
 )
 
(define (generateReply0 input)
   (map cog-bind (cog-outgoing-set (findQueryPatterns input)))
 )
 (define (generateReply1 input)
   (map cog-bind (cog-outgoing-set (cog-recognize (genQueryPattern input))))
 )
(define (generateSRAIReply input)
(display "generateSRAIReply:")(pretty-print input)(newline)
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
 (cog-delete-recursive (mapConceptualizeString input))
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

;; ELIZA-ish test data to match against

(BindLink
  (ListLink (ConceptNode "SORRY"))
  (ListLink
    (ExecutionOutputLink
      (RandomChoiceLink
        (ListLink
          (NumberNode 1/4)
          (NumberNode 1/4)
          (NumberNode 1/4)
          (NumberNode 1/4))
        (ListLink
          (ListLink
            (ConceptNode "PLEASE")
            (ConceptNode "DON'T")
            (ConceptNode "APOLOGIZE.."))
          (ListLink
            (ConceptNode "APOLOGIES")
            (ConceptNode "ARE")
            (ConceptNode "NOT")
            (ConceptNode "NECESSARY.."))
          (ListLink
            (ConceptNode "I'VE")
            (ConceptNode "TOLD")
            (ConceptNode "YOU")
            (ConceptNode "THAT")
            (ConceptNode "APOLOGIES")
            (ConceptNode "ARE")
            (ConceptNode "NOT")
            (ConceptNode "REQUIRED.."))
          (ListLink
            (ConceptNode "IT")
            (ConceptNode "DID")
            (ConceptNode "NOT")
            (ConceptNode "BOTHER")
            (ConceptNode "ME.")
            (ConceptNode "PLEASE")
            (ConceptNode "CONTINUE..")))))))
			
(BindLink
  (ListLink
    (ConceptNode "I")
    (ConceptNode "CAN")
    (ConceptNode "NOT"))
  (ListLink
    (ConceptNode "WHY")
    (ConceptNode "IS")
    (ConceptNode "THAT?")))
	
(BindLink
  (ListLink
    (ConceptNode "I")
    (ConceptNode "REMEMBER")
    (GlobNode "$star1_1"))
  (ListLink
    (ExecutionOutputLink
      (RandomChoiceLink
        (ListLink
          (NumberNode 1/5)
          (NumberNode 1/5)
          (NumberNode 1/5)
          (NumberNode 1/5)
          (NumberNode 1/5))
        (ListLink
          (ListLink
            (ConceptNode "DO")
            (ConceptNode "YOU")
            (ConceptNode "OFTEN")
            (ConceptNode "THINK")
            (ConceptNode "OF")
            (GlobNode "$star1_1")
            (ConceptNode "?."))
          (ListLink
            (ConceptNode "DOES")
            (ConceptNode "THINKING")
            (ConceptNode "OF")
            (GlobNode "$star1_1")
            (ConceptNode "BRING")
            (ConceptNode "ANYTHING")
            (ConceptNode "ELSE")
            (ConceptNode "TO")
            (ConceptNode "MIND?."))
          (ListLink
            (ConceptNode "WHY")
            (ConceptNode "DO")
            (ConceptNode "YOU")
            (ConceptNode "REMEMBER")
            (GlobNode "$star1_1")
            (ConceptNode "JUST")
            (ConceptNode "NOW?."))
          (ListLink
            (ConceptNode "WHAT")
            (ConceptNode "IN")
            (ConceptNode "THE")
            (ConceptNode "PRESENT")
            (ConceptNode "SITUATION")
            (ConceptNode "REMINDS")
            (ConceptNode "YOU")
            (ConceptNode "OF")
            (GlobNode "$star1_1")
            (ConceptNode "?."))
          (ListLink
            (ConceptNode "WHAT")
            (ConceptNode "IS")
            (ConceptNode "THE")
            (ConceptNode "CONNECTION")))))))
			
;;TEST CASE ONE
;; I get the following error sequence :
;;scheme@(guile-user)> (answerInput "SORRY")
;;ERROR: In procedure opencog-extension:
;;ERROR: Throw to key `C++-EXCEPTION' with args `("cog-bind" "invalid outgoing set index 1 (/home/adminuser/OPENCOG/atomspace/opencog/atomspace/Link.h:176)")'.

;; TEST CASE TWO
;;scheme@(guile-user) [1]> ,q
;;scheme@(guile-user)> (answerInput "I CAN NOT")
;;((SetLink
;;   (ListLink
;;      (ConceptNode "WHY")
;;      (ConceptNode "IS")
;;      (ConceptNode "THAT?")
;;   )
;;)
;;)
;; In the second case it matches an explicit pattern with one output but then guile doesn't come back to the REPL prompt.

;;TEST CASE THREE
;; (answerInput "I REMEMBER LAST WEEK")
;; Same result as TEST CASE ONE

	