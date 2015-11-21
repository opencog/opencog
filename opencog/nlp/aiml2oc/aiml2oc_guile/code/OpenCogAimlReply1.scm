(use-modules (ice-9 pretty-print))
(use-modules (ice-9 readline))
(use-modules (opencog atom-types))
(use-modules (srfi srfi-13))
(use-modules (srfi srfi-1))

(define (mapConceptualizeString input)
   (ListLink 
    (map (lambda (x) (ConceptNode x )) (string-split input #\ )))
   )
   
(define (mapConceptualizeStringList inputList)
   (ListLink 
    (map (lambda (x) (ConceptNode x )) inputList))
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
  ;;(display "generateReply:")(pretty-print input)(newline)
  (let ((queryPatterns  (findQueryPatterns  input)) (outset '()) )
   (begin
     ;;(display "generateReply-queryPatterns:")(pretty-print queryPatterns)(newline)
	 (set! outset (cog-outgoing-set queryPatterns))
	 ;;(display "generateReply-outset:")(pretty-print outset)(newline)
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
;;(display "generateSRAIReply:")(pretty-print input)(newline)
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
			;(gc) (gc) (gc)) ; MUST GC OR ELSE DELTE ATOMSPACE STICKS AROUND!!
			)
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
 ;;(cog-delete-recursive (mapConceptualizeString input))
 (pop-atomspace)
)

(define (sraiAnswer input)
 (let ( (ans (list)) (ansList (list)) )
  (begin
    (push-atomspace)
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
	(pop-atomspace)
	ans
   )
  )
)

(define (answerInput2 input)
 (let ( (ans (list)) (ansList (list)) )
  (begin
	(cog-push-atomspace)

	(set! ans (generateReply input))

	(set! ansList (cog-get-all-nodes (car ans)))
	(if (equal? (cog-name (car ansList)) "AIMLSRAI")
	    (begin
		    ;;(display "ans1:")(pretty-print ans)(newline)
			(set! ans (sraiAnswer (cdr ansList)))
			;;(display "ans2:")(pretty-print ans)(newline)
		)
	)
	 (cog-pop-atomspace)
	;;(display "ans:")(pretty-print ans)(newline)
	;;(display "ansList:")(pretty-print ansList)(newline)
	ans
   )
 )
)

(define (answerInput3 input)
 (let ( (ans (list)) (ansList (list)) (frontToken (list)))
  (begin
	(push-atomspace)
	(set! ans (generateReply input))
	;;(display "ans:")(pretty-print ans)(newline)
	(if (equal? (length ans) 0) (set! ans (generateReply "NOREPLY")) )
	
	(set! ansList (cog-get-all-nodes (car ans)))
	(set! frontToken (cog-name (car ansList)))
	;;(display "frontToken:")(pretty-print frontToken)(newline)
	(if (equal? frontToken "AIMLSRAI")
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

(define personalizeList
  (lambda (plist mode)
   (let ((ans (list)))
    (begin
	   (if (null? plist) 
		   (set! ans '() )
		   (begin
			   ;;(display "mode:")(pretty-print mode)(display "  plist:")(pretty-print plist)(newline)
			   (if (equal? mode 0) 
				   (if (equal? (car plist) "BEGINPERSON")
					 (set! ans (personalizeList (cdr plist) 1)) ;; turn on change
					 (set! ans (append (list (car plist)) (personalizeList (cdr plist) 0))) ;; don't flip current token
					)
				  )
				(if (equal? mode 1)
				   (if (equal? (car plist) "ENDPERSON")
					 (set! ans (personalizeList (cdr plist) 0)) ;turn it off
					 (set! ans (append (list (simplePronounFlip (car plist))) (personalizeList (cdr plist) 1)) );; return flipped current token
					)
				)
			)
		)
		;; Clean up and missed markers or unfilled globs
	   (if (and (not(null? ans))(equal? (car ans)  "BEGINPERSON")) (set! ans (cdr ans)))
	   (if (and (not(null? ans))(equal? (car ans)  "ENDPERSON")) (set! ans (cdr ans)))
	   (if (and (not(null? ans))(equal? (car ans)  "$star1_1")) (set! ans (cdr ans)))
	   (if (and (not(null? ans))(equal? (car ans)  "$star2_1")) (set! ans (cdr ans)))
	)
    ans
   )
  )
)
 
(define simplePronounFlip
  (lambda (a)
	(let ( (ans a))
	  (if (equal? a "I") (set! ans "YOU"))
	  (if (equal? a "YOU") (set! ans "I"))
	  (if (equal? a "MY") (set! ans "YOUR"))
	  (if (equal? a "I'M") (set! ans "YOU ARE"))
	  (if (equal? a "MYSELF") (set! ans "YOURSELF"))
	  (if (equal? a "YOURSELF") (set! ans "MYSELF"))
	  (if (equal? a "ME") (set! ans "YOU"))
	  (if (equal? a "YOUR") (set! ans "MY"))
	  (if (equal? a "ARE") (set! ans "AM"))
	 ans 
	)
  )
)

(define flipSelf
  (lambda (sent split N)
   (let ((ans '()))
   (begin
    ;;(display "flipSelf sent:")(pretty-print sent)(display "  split:")(pretty-print split)(display "  N:")(pretty-print N)(newline)
    (if (null? sent) 
	     (set! ans '())
		(if (<= N split)
		  (set! ans (append (list (car sent)) (flipSelf (cdr sent) split (+ N 1)) ))
		  (if (equal? (car sent) "I") 
			(set! ans (append (list "ME") (flipSelf (cdr sent) split (+ N 1)) ))
			(set! ans (append (list (car sent)) (flipSelf (cdr sent) split (+ N 1)) ) )
		   )
	     )
      )
	)
	ans
  )
  )
)

(define guessSelfPronoun
;; Usually use "ME" if an object or after a verb, so guess on that flip after the half-way mark
;; true operation would be to flip on a parse of the output
;; or use bigrams and trigrams to get the proper "sounding" output
  (lambda (sent)
  (flipSelf sent (* (length sent) 0.47) 0)	
  )
)
 
(define extractStringList
 ;; Convert ((SetLink (ListLink (ConceptNode "a") ....(ConceptNode "z"))) to ("a" .... "z")
	 (lambda (a)
	  (let ((set1 (car a)) (set2 '()) (set3 '()) (set4 '()))
		  (begin
			(set! set2 (cog-outgoing-set set1))
			(set! set3 (car set2))
			(set! set4 (cog-outgoing-set set3))
			;;(map cog-name   (cog-outgoing-set (car (cog-outgoing-set (car a)))))
			(map cog-name  set4)
		   )
      )
	 )
 )

(define (respondToInput input) 
	(guessSelfPronoun (personalizeList (extractStringList (answerInput3 input)) 0 ))
)

(define punctuation (string->list "!\"#$%&\\'()*+,-./:;<=>?@[\\]^_`{|}~"))

(define (cleanText text)
  ;;"Replace punctuation characters from TEXT with a space character"
  ;;(string-map (lambda (char) (if (list-index punctuation char) #\space char)) text))
(string-map (lambda (char) (if (member char punctuation) #\space char)) text))

(define print-with-spaces 
  ;; Print the elements of a list separated by spaces.
  (lambda (outlist)
    (if (not (null? outlist))
	  (begin 
	     (if (not (equal? (car outlist) " ")) (format #t "~A " (car outlist)) )
		 (print-with-spaces (cdr outlist))
		 )
	   (format #t "\n"))
	   )
)

(define chatty 
  ;; Respond to user input
  (lambda ()
    (do ((done #f))
      (done)
      ;;(format #t "chatty> ")
      (let* ((input (readline "user: "))
             (response (respondToInput (cleanText (string-upcase input)))))
        (format #t "chatty> ")	(print-with-spaces response)
        (when (equal? response '(good bye)) 
		      (set! done #t))
	   )
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
;(respondToInput "HOW ARE YOU TONIGHT")
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
