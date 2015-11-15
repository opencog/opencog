(use-modules (sxml simple))
(use-modules (ice-9 pretty-print))
(use-modules (opencog atom-types))
;;==============================================================
;;http://stackoverflow.com/questions/5546552/scheme-recursive-function-to-compute-all-possible-combinations-of-some-lists
;;==============================================================

(define concat-map
  (lambda (ls f)
    (cond
      [(null? ls) '()]
      [else (append (f (car ls)) (concat-map (cdr ls) f))])))

(define combine-cross
  (lambda (xs ys)
    ;;(display "at-combine-cross ")(display xs) (display ys) (newline)
    (concat-map xs (lambda (x)
                     (map (lambda (y) (append x y)) ys)))))

(define (nth n l)
  (if (or (> n (length l)) (< n 0))
    (error "Index out of bounds.")
    (if (eq? n 0)
      (car l)
      (nth (- n 1) (cdr l)))))
	  
; generate a sequence of variable nodes of the form "<STAR><INDEX>_<COUNTER>" with <COUNTER> incrementing until num
(define star2 
	(lambda (l index num)
	 (let ((ans l))

		 (if (= num 0) (set! ans (list) ))
		 (if (> num 0)
			(set! ans (append 
		                       (star2 l index (- num 1))
		              
      				    (list
 					(list
					'GlobNode
		                       (string-append 
		                                 l 
		                                (number->string index) 
		                                "_" 
		                                (number->string num)
					)))
                         ) ) )
         ans)
	)
)


(define countStars
 (lambda (l)
 ;;(display "countStars:")(display l) (newline)
    (if (null? l) 0
       (if (or (equal? (car l) '* )(equal? (car l) "*")) (+ (countStars (cdr l)) 1)
	     (if (list? (car l)) (+ (countStars (car l)) (countStars (cdr l)) )
                               (countStars (cdr l))
		 )
       )
    )
  )
)

(define countStarsInPatterns
  (lambda (l)
   ;;(display "countStarsInPatterns:")(display l) (newline)
    (if (null? l) 0
	    (if (equal? (car l) 'pattern) (countStars (cdr l))
		     (countStarsInPatterns (cdr l))
		))))

(define getTag
  (lambda (a l)
   (begin
    (if (null? l) (list)
	    (if (equal? (caar l) a) (car l)
		     (getTag a (cdr l))
		)))))
		
		
	
(define mstar1a
   (lambda (l index expansion)
    (list
     (star2 l index expansion)
    ) 
  )
)

(define pattern4oc
   (lambda (l starCount expansion)
     (let ((ans l)  (rest (list(list))) )
      (begin
       ;;(display "enter p3oc ")(display l)(display starCount)(newline)
       (if (null? l) (begin (set! ans (list(list))) )
           (if (equal? (car l) '*)
                 (begin (set! ans (mstar1a "$star" starCount expansion))
                  (set! rest (pattern4oc (cdr l) (+ starCount 1) expansion) )   
                  ;;(display "return1:")(display rest)(newline)
                  )
                 (begin (set! ans (list (list (list 'ConceptNode (car l)))) )
                  (set! rest (pattern4oc (cdr l) starCount expansion) )
                  ;;(display "return2:")(display rest)(newline)
                  )
          ))
         ;;(display "fin p3oc ")(display ans)(display rest)(newline)
         )
       (combine-cross ans rest)
      )
    )
)
(define pattern2oce
  (lambda (a expansion)
   (display a)
   (newline)
   (let* ((l1 (map string->symbol (string-split a #\ ) ))
         )
	(append (list 'ListLink) (car (pattern4oc l1 1 expansion)) )
            
   )
  )
)


(define aimltag4oc
  (lambda (a expansion)
   (let ((ans a))
     (display "aimltag4oc:") (display a) (newline)
     (if (string? a)             (set! ans (list 'ConceptNode a))
            ( begin
	     (if (equal? (car a) 'pattern)  (set! ans (pattern2oce (cadr a) expansion)) )
	     (if (equal? (car a) 'template) (set! ans (template4oc (cdr a) expansion)) )
	     (if (equal? (car a) 'think)    (set! ans (list (list 'ConceptNode "AIMLTHINK"  ) (cdr a))) )
	     (if (equal? (car a) 'srai)     (set! ans (list (list 'ConceptNode "AIMLSRAI"  ) (map aimltag4oc (cdr a) expansion))) )
	     (if (equal? (car a) 'star)     (set! ans (list (list 'ConceptNode "AIMLSTAR"  ) (mstar1a "$star" 1 expansion))) )
	     (if (equal? (car a) 'random)   (set! ans (randomExpansion (cdr a) ) )) 
		 
             )
     )
     ans
   )
  )
)



(define bindPatternStar 
 (lambda (pattern glob index count)
  (if (null? pattern) (list)
     (if (equal? (car pattern) '*)
	     (begin
		    
		    (if (equal? index (+ count 1))
			  (begin
			     ;; Insert the glob here
			     (append glob (bindPatternStar (cdr pattern) glob index (+ count 1)))
			   )
			   (begin
			     ;; Not our glob, someone else glob
			     (append (list '*) (bindPatternStar (cdr pattern) glob index (+ count 1)))
			   )
			 )  
		  )
		 (begin
		      (append (list (car pattern)) (bindPatternStar (cdr pattern) glob index count))
		 )
	  )
	)
 )
)

(define bindPatternStar 
 (lambda (pattern glob index count)
 ;;(display "bindPatternStar-pattern:")(display pattern) (display "   -glob:")(display glob)(display "   -index:")(display index)(display "   -count:")(display count)(newline)
  (if (null? pattern) (list)
     (if (or (equal? (car pattern) '*)(equal? (car pattern) "*"))
	     (begin
		    
		    (if (equal? index (+ count 1))
			  (begin
			     ;; Insert the glob here
			     (append glob (bindPatternStar (cdr pattern) glob index (+ count 1)))
			   )
			   (begin
			     ;; Not our glob, someone else glob
			     (append (list '*) (bindPatternStar (cdr pattern) glob index (+ count 1)))
			   )
			 )  
		  )
		 (begin
		      (append (list (car pattern)) (bindPatternStar (cdr pattern) glob index count))
		 )
	  )
	)
 )
)
(define bindPatternStarN 
 (lambda (pattern globList count)
 ;;(display "bindPatternStar-patternN:")(display pattern) (display "   -glob:")(display globList)(display "   -count:")(display count)(newline)
  (if (null? pattern) (list)
     (if (or (equal? (car pattern) '*)(equal? (car pattern) "*"))
	     (begin
			 (append (nth count globList) (bindPatternStarN (cdr pattern) globList (+ count 1)))
		  )
		 (begin
		      (append (list (car pattern)) (bindPatternStarN (cdr pattern) globList count))
		 )
	  )
	)
 )
)

(define cleanNulls
	(lambda(a)
	   (if (null? a) (list)
		(if (and (string? (car a))(string-null? (car a)))
		   (cleanNulls (cdr a))
		   (append (list (car a)) (cleanNulls (cdr a)))
		 )
		)
	)
)

(define atomize
 (lambda (a)
   (let ((strList (map (lambda (x) (if (string? x) (string-split x #\ ) x)) a) ))
    
	(flatList (map (lambda (x) (if (string? x) (string->symbol x ) x)) (cleanNulls strList )))
	)
))

(define atomize0
 (lambda (a)
   (let ((strList (map (lambda (x) (if (string? x) (string-tokenize x (char-set-complement (->char-set ",\"")) ) x)) a) ))
    
	(flatList (map (lambda (x) (if (string? x) (string->symbol x ) x)) (cleanNulls strList )))
	)
))

(define upperlist
 (lambda (inlist)
   (map (lambda (x) (if (string? x) (string-upcase x) x)) inlist)
  )
 )

(define flatList
  (lambda (e)
    (cond ((pair? e) `(,@(flatList (car e)) ,@(flatList (cdr e))))
          ((null? e) '())
          (else (list e)))))
		  
;(atomize (list 'pattern "a b c"))
;(countStarsInPatterns (atomize (list 'pattern "a b c * x * y")))

(define conceptualizePattern
  (lambda (pattern)
   (if (null? pattern) (list)
     (if (list? (car pattern)) 
	    (append (list (car pattern)) (conceptualizePattern (cdr pattern)))
		(if (and (string? (car pattern))(string-null? (car pattern))) 
		    (conceptualizePattern (cdr pattern))
			(append (list (list 'ConceptNode  (car pattern))) (conceptualizePattern (cdr pattern)))
		 )
	  )
	)
  )
)



(define extractIndex
   ;; either (star) or (star (@ (index 2)))
   (lambda (template)
    (let ((ans 1))
     (begin
	   (if (null? (cdr template)) (set! ans 1)
	     (set! ans (string->number (cadr (cadr ( cadr template)))) )
	   )
	 )
	 ans
	 )
   )
)

;(extractIndex (list 'star))
;(extractIndex (list 'star (list '@ (list 'index 2))) )

(define (randomChoice0 . choice-list)
	(nth (random (length choice-list)) choice-list)
) 
 
(define (randomChoice . choice-list)
    (display "randomChoice")(display choice-list)(newline)
	(car choice-list)
)

(define (fixStars clist)
  ; should be (ConceptNode "text") (ConceptNode (star)) or (ConceptNode (star (index 2)))
  (if (string? (car(cdr clist)))
		(list (car clist)(string-upcase (car (cdr clist))))
		(if (list?  (car(cdr clist)))
			(list 'GlobNode "$star2_1")
			(list 'GlobNode "$star1_1")
		)
   )
)

(define liExpand 
 (lambda (expression)
   (append (list 'ListLink ) (map fixStars (conceptualizePattern (atomize (cdr expression)))))
 )
)

(define genWeightList
 (lambda (n w)
  (if (eq? n 0) 
	(list) 
	(append (list (list 'NumberNode (number->string w))) (genWeightList (- n 1) w))
  )
 )
)

(define generateRandomWeights
 (lambda (options)
  (let ( (weight (/ 1 (length options))))
	    (append (list 'ListLink) (genWeightList (length options) weight))
	   )
   )
  )
 
  
(define randomExpansion
  (lambda (expression)
   (let ( (core (map liExpand expression)) )
     (list  (list 'RandomChoiceLink  (generateRandomWeights core)  (append (list 'ListLink) core)))
	)
  )
)
(define randomExpansion1
  (lambda (expression)
   (let ( (core (map liExpand expression)) )
     (list (list 'ExecutionOutputLink (list 'GroundedSchemaNode "scm:randomChoice" (append (list 'ListLink) core))))
	)
  )
)
(define randomExpansion0
  (lambda (expression)
   (let ( (core (map liExpand expression)) )
     (list  (car core))
	)
  )
)
(define bindTemplateStar 
 (lambda (template glob index)
 ;;(display "bindTemplateStar:")(display template) (display "  -glob:")(display glob)(display "  -index:")(display index)(newline)
   (let ( (ans (list)) (curindex 1) (temp template) (skip 'f))
     (begin
	   (if (null? template) (set! ans (list(list)))
				(begin
				   (if (and (list? template)(or (string? (car template))(symbol? (car template))))             
					   (begin
					    ;;(if (string? (car template)) (set! ans (list (list 'PhraseNode (car template)))) ) 
						;; if a string or symbol give the appropriate Concept node form
					    (if (string? (car template)) (set! ans (conceptualizePattern (upperlist (atomize (list(car template)))))) ) 
					    (if (symbol? (car template)) (set! ans (list (list 'ConceptNode (car template)))) ) 
					   )
					 )
					 ;; Are they specific tags
					(if (equal? (car template) 'template) (set! ans  (list) ) )
					(if (equal? (car template) 'think)    (set! ans (list (list 'ConceptNode "AIMLThink" )) )) 
					(if (equal? (car template) 'srai)     (set! ans (list (list 'ConceptNode "AIMLSRAI"  )) ))
					(if (equal? (car template) 'star)    
					  (begin
					     ;; If it is a star then get the embedded index if any
					     ;; should change curindex for this star if applicable
						 (set! curindex (extractIndex template))
						 ;; if the index matches insert the glob, otherwise leave for another pass
					     (if (equal? curindex index)
						     (set! ans glob)
                             ;;(set! ans (list (list (car template))) )
                             (set! ans (list template)) 
                          )	
                        (set! skip 't)						  
					  )
				    )
					(if (equal? (car template) 'sr)    
					  (begin
					     ;; If it is a star then get the embedded index if any
					     ;; should change curindex for this star if applicable
						 (set! curindex (extractIndex template))
						 ;; if the index matches insert the glob, otherwise leave for another pass
					     (if (equal? curindex index)
						     (set! ans (append (list (list 'ConceptNode "AIMLSRAI")) glob))
                             ;;(set! ans (list (list (car template))) )
                             (set! ans (list template)) 
                          )	
                        (set! skip 't)						  
					  )
				    )
					
					(if (equal? (car template) 'random)    
					  (begin
					    (set! ans (randomExpansion (cdr template))) 
						(set! skip 't)
					  )
					 )
					;; If it is a list then embed the appropriate star inside the rest
				   (if (list? (car template))
				      (set! ans (bindTemplateStar (car template) glob index)) )
				)
		)
	 )
	 ;;(display "bindTemplateStar-ans:")(display ans) (newline)
	 ;;(display "bindTemplateStar-template:")(display temp) (newline)
	 (if (null? temp)
		(list)
		(if (equal? skip 'f) 
		   (append ans (bindTemplateStar (cdr temp) glob index))
		   ans
		)
	  )
	)
 )
)

(define bindTemplateStarN 
 (lambda (template globList)
 ;;(display "bindTemplateStarN:")(display template) (display "  -glob:")(display glob)(display "  -index:")(display index)(newline)
   (let ( (ans (list)) (curindex 1) (temp template) (skip 'f))
     (begin
	   (if (null? template) (set! ans (list(list)))
				(begin
				   (if (and (list? template)(or (string? (car template))(symbol? (car template))))             
					   (begin
					    ;;(if (string? (car template)) (set! ans (list (list 'PhraseNode (car template)))) ) 
						;; if a string or symbol give the appropriate Concept node form
					    (if (string? (car template)) (set! ans (conceptualizePattern (atomize (list(car template))))) ) 
					    (if (symbol? (car template)) (set! ans (list (list 'ConceptNode (car template)))) ) 
					   )
					 )
					 ;; Are they specific tags
					(if (equal? (car template) 'template) (set! ans  (list) ) )
					(if (equal? (car template) 'think)    (set! ans (list (list 'ConceptNode "AIMLTHINK"  ))   )) 
					(if (equal? (car template) 'srai)     (set! ans (list (list 'ConceptNode "AIMLSRAI"  )) ))
					(if (equal? (car template) 'star)    
					  (begin
					     ;; If it is a star then get the embedded index if any
					     ;; should change curindex for this star if applicable
						 (set! curindex (extractIndex template))
						 ;; index the rightGlob
						 (set! ans (list template))
					     (if (>= curindex 1)     (set! ans (nth (- curindex 1) globList))) 
					     (if (equal? curindex 0) (set! ans (list template)) )

                        (set! skip 't)						  
					  )
				    )
					(if (equal? (car template) 'sr)    
					  (begin
					     ;; If it is a star then get the embedded index if any
					     ;; should change curindex for this star if applicable
						 (set! curindex (extractIndex template))
						 ;; index the rightGlob
						 (set! ans (list template))
					     (if (>= curindex 1)     (set! ans (append (list (list 'ConceptNode "AIMLSRAI")) (nth (- curindex 1) globList))) ) 
					     (if (equal? curindex 0) (set! ans (list template)) )

                        (set! skip 't)						  
					  )
				    )
					
					(if (equal? (car template) 'random)    
					  (begin
					    (set! ans (randomExpansion (cdr template))) 
						(set! skip 't)
					  )
					 )
					
					;; If it is a list then embed the appropriate star inside the rest
				   (if (list? (car template))
				      (set! ans (bindTemplateStarN (car template) globList )) )
				)
		)
	 )
	 ;;(display "bindTemplateStarN-ans:")(display ans) (newline)
	 ;;(display "bindTemplateStarN-template:")(display temp) (newline)
	 (if (null? temp)
		(list)
		(if (equal? skip 'f) 
		   (append ans (bindTemplateStarN (cdr temp) globList ))
		   ans
		)
	  )
	)
 )
)


;(bindTemplateStar (list 'template "So you are saying every"  (list 'star)  "are also"  (list 'star (list '@ (list 'index 2)))) (list 'star1_1) 1)
;(bindTemplateStar (list 'a 'b (list 'star) 'c) (list 'star1_1) 1)
;(bindTemplateStar (list 'a 'b (list 'star) 'c) (list 'star1_1 'star1_2) 1)
;(bindTemplateStar (list 'a 'b (list 'star) 'c) (list 'star2_1) 2)

;; This system is modified for using the GlobNode, so only one star expansion per aiml star										
(define processCategory
	(lambda (a)
	 (let
	    ( 
			(numStars (countStarsInPatterns (atomize (getTag 'pattern a))))  
			(ans (list)) 
			(starlist (list))
			(starlist2 (list)) 
			(crossStars (list)) 
		)
		
		(begin
		    ;; 0 == one to one transcription
			;;(display "processCategory countStarsInPatterns:")(display numStars) (newline)
		   (if (equal? numStars 0)	
		        (begin
		            (set! ans (list (list 'BindLink
										          (append (list 'ListLink) (conceptualizePattern (bindPatternStar  (atomize (cdr (getTag 'pattern a))) (list) 1 -1)))
										          (append (list 'ListLink) (bindTemplateStar (getTag 'template a) (list) 1) )
												 ))
					 )
				)
		       (if (equal? numStars 1)
			         (begin
					   ;; single star processing
					   ;; generate glob pattern for stars
					    (set! starlist   (list  
												;;(star2 "$star" numStars 0)
												(star2 "$star" numStars 1)
												;;(star2 "$star" numStars 2)
												;;(star2 "$star" numStars 3)
											)) 
					  ;; map star glob patterns into both pattern and template
						(set! ans (map (lambda (starglob) 
										  (list (list 'BindLink
										          ;;(append (list 'ListLink) (conceptualizePattern (bindPatternStar  (atomize (car(cdr (getTag 'pattern a)))) starglob 1 0)))
										          (append (list 'ListLink) (conceptualizePattern (bindPatternStar  (atomize (cdr (getTag 'pattern a))) starglob 1 0)))
										          (append (list 'ListLink) (bindTemplateStar (getTag 'template a) starglob 1) )
												 ))
						                )
										starlist) )

					 )
			    (if (equal? numStars 2)
				      (begin
					    ;; double star processing
					    (set! starlist   (list  
												;;(star2 "$star" numStars 0)
												(list (star2 "$star" 1 1))
												;;(list (star2 "$star" 1 2))
												;;(list (star2 "$star" 1 3))
											)) 
					    (set! starlist2   (list  
												;;(star2 "$star" numStars 0)
												(list (star2 "$star" 2 1))
												;;(list (star2 "$star" 2 2))
												;;(list (star2 "$star" 2 3))
											))
						(set! crossStars (combine-cross	starlist starlist2))		
					  ;; map star glob patterns into both pattern and template
						(set! ans (map (lambda (starglobPair) 
										  (list (list 'BindLink
										          ;;(append (list 'ListLink) (conceptualizePattern (bindPatternStar  (atomize (car(cdr (getTag 'pattern a)))) starglob 1 0)))
										          (append (list 'ListLink) (conceptualizePattern (bindPatternStarN  (atomize (cdr (getTag 'pattern a))) starglobPair 0)))
										          (append (list 'ListLink) (bindTemplateStarN (getTag 'template a) starglobPair) )
												 ))
						                )
										crossStars) )
					  )
					  (begin
					     (set! ans (list)) ;; Not ready yet condition
					  )
				)
			   )	
		   )
	    )
		ans
	  )
	)
)

(define aiml4oc
  (lambda (a)
    ;;(display "aiml4oc:")(display a) (newline)
    (let ((ans a))
    (if (equal? (car a) 'aiml) (set! ans (map aiml4oc (cdr a))))
    (if (equal? (car a) 'category) (set! ans (processCategory (cdr a))) )
    ans
     )
   )
) 

(define processAimlFile 
  (lambda (filename)
   (aiml4oc (cadr (call-with-input-file filename (lambda (port) (xml->sxml port #:trim-whitespace? #t)) )) )
  )
)
(define showAimlFile 
    (lambda (input-file output-file)
       (call-with-output-file output-file
		(lambda (output-port)
		   (map (lambda (x)	(printLinks  x output-port)) (call-with-input-file input-file (lambda (port) (xml->sxml port #:trim-whitespace? #t))) )
		 )
		)
	)
)
			 

;;(processAimlFile "/home/adminuser/testaiml2.aiml")



(define printLinks
  (lambda (l port)
    (if (pair? l)
		(if (list? (car l))  (begin  (printLinks (car l) port) (printLinks (cdr l ) port))
			(pretty-print l port)
		)
		;;(pretty-print l port)
	)
  )
)

(define processAimlFile2OC
   (lambda (input-file output-file)
       (call-with-output-file output-file
		(lambda (output-port)
		   (map (lambda (x)	(printLinks  x output-port)) (processAimlFile input-file))
		 )
		)
	)
)

(define processAndLoadAIML 
   (lambda (input-file output-file)
     (processAimlFile2OC input-file output-file)
	 (primitive-load output-file)
    )
)
;;================================================================
;; What we have all been waiting for ...
;;================================================================
;(processAimlFile2OC "/home/adminuser/testaiml2.aiml" "/home/adminuser/testaiml2.oc")	
;(processAimlFile2OC "/home/adminuser/share/knowledge.aiml" "/home/adminuser/share/knowledge.scm")	
	
;(processAndLoadAIML "/home/adminuser/share/knowledge.aiml" "/home/adminuser/share/knowledge.scm")	

;(processAndLoadAIML "/home/adminuser/share/test1.aiml" "/home/adminuser/share/test1.scm")	

;;(bindPatternStar (list 'a 'b '* 'c) (list 'star1_1) 1 0)
;;(bindPatternStar (list 'a 'b '* 'c) (list 'star1_1 'star1_2) 1 0)
 
;;(processCategory (list  
;;                        ( list 'pattern 'ALL '* 'ARE '* )
;;						(list 'template "So you are saying every"  (list 'star)  "are also"  (list 'star (list '@ (list 'index 2))))
;;						)
;;						)
;;(processCategory (list  
;;                        (list 'pattern 'I '* 'you )
;;						(list 'template "I"  (list 'star) "you too"  ))
;;						)
						
;;(countStarsInPatterns (getTag 'pattern (list  
;;                        (list 'pattern 'I '* 'you )
;;						(list 'template "I"  (list 'star) "you too"  ))
;;						))
						
;;(countStarsInPatterns (getTag 'pattern (list  
;;                        ( list 'pattern 'ALL '* 'ARE '* )
;;						(list 'template "So you are saying every"  (list 'star)  "are also"  (list 'star (list '@ (list 'index 2))))
;;						)
;;						))

;(load "/home/adminuser/share/code/OpenCogAimlProcess.scm")
;(load "/home/adminuser/share/code/OpenCogAimlPre2.scm")