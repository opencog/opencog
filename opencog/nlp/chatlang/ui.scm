#!/usr/local/bin/guile -s
!#

(use-modules (ice-9 readline)(ice-9 regex)) 

(define (s str) (quasiquote ,str))

;(define (b str) (quasiquote ,str)) ; placeholder for behaviour

(define (cr strRule strSay)
	(define strBuilder "")
	(define tmp "")	
	(define wordtype "")
	(define x "")
	(define flag_or-choice 0) (define flag_proper-names 0) (define flag_unordered-matches 0)
	(define intCounter 0) (define intCounter2 0)
	(define pos_flag 1)
	(define (my-interpret var) ; funciton
			;;
			;; Treat each word separately
			;;
        	      	; (if (string-match "~" var) (set! wordtype "concept") (set! wordtype "lemma"))  
			(cond 
				((string-match "^[~][[:alnum:]]" var) (set! wordtype "concept")) ; concept
      				((string-match "^['][[:alnum:]]" var) (set! wordtype "word")) ; word
				((string-match "[[:alnum:]]~mainsubject" var) (begin ; mainsubject
					(set! wordtype "main-subj") 
					(if (string-match "~mainsubject" var) (set! var (regexp-substitute #f (string-match "~mainsubject" var)  'pre "" 'post) ) )
				))   ;Ben~mainsubject	^[[:alnum:]]*~mainsubject
				((string-match "[[:alnum:]]~mainverb" var) (begin ; mainverb
                                        (set! wordtype "main-verb")
                                        (if (string-match "~mainverb" var) (set! var (regexp-substitute #f (string-match "~mainverb" var)  'pre "" 'post) ) )
                                ))
				((string-match "[[:alnum:]]~mainobject" var) (begin ; mainobject
                                        (set! wordtype "main-obj")
                                        (if (string-match "~mainobject" var) (set! var (regexp-substitute #f (string-match "~mainobject" var)  'pre "" 'post) ) )
                                ))  
				; part of speech
				((string-match "^[[:alpha:]]+~[[:alpha:]]+$" var) (begin ; part of speech
                                        (set! wordtype "pos")
					(set! var (regexp-substitute #f (string-match "[~]" var)  'pre "\" \"" 'post) )

                                ))
      				((string-match "^([[:digit:]]+~[[:digit:]]+|[*])|[*]$" var) (begin ; wildcard
					(if (string-match "^[[:digit:]]+[*]$" var) (set! var (regexp-substitute #f (string-match "[*]" var)  'pre "" 'post) ) )
					(set! wordtype "wildcard")
				))
				((string-match "^[[:alnum:][:space:]]*[[:space:]]<[[:space:]][[:alnum:][:space:]]*$" var) (begin ; start-with <
					(set! var (string-trim (regexp-substitute #f (string-match "^.*<" var)  'pre "" 'post) ) ) ; discard everything before the '<'
					(set! var (regexp-substitute #f (string-match "[[:space:]]*" var)  'pre "\" \"" 'post) )
                                        (set! wordtype "start-with")
                                ))
				((string-match ".*[[:space:]]*<[[:space:]]*.*" var) (begin ; start-with <
                                        ;(set! var (string-trim (regexp-substitute #f (string-match "<" var)  'pre "" 'post) ) ) ; discard everything before the '<'
                                        ;(set! var (regexp-substitute #f (string-match "[[:space:]]*" var)  'pre "" 'post) )
                                        (set! wordtype "start-with")
                                ))
				
				(else (set! wordtype "lemma"))
			)
		
			(string-append "(" wordtype " \"" var "\"" ") " )

			;(for-each (lambda (x) (set! result (string-append result x)))  )
			;result	
	)

	;;
	;; Process groups of words
	;;
	(define (my-interpret-group x name leftMatch rightMatch flag)
                ( (or (string-match leftMatch x) (equal? flag 1) )
                	(if (string-match leftMatch tmp) (set! tmp (regexp-substitute #f (string-match leftMatch tmp)  'pre "" 'post) ) )
                        (if (string-match rightMatch tmp) (set! tmp (regexp-substitute #f (string-match rightMatch tmp)  'pre "" 'post) ) )
                      	 ;(display  intCounter2)
                       	(if (equal? intCounter2 0)
                        	(begin
                                	(set! strBuilder (string-append strBuilder "(" name " "))
                                        (if (equal? tmp "") (display "") (set! strBuilder (string-append strBuilder "\""  tmp "\" ")) )
                                        (set! intCounter2 1)
                                )
                                (if (not (equal? tmp ""))  (set! strBuilder (string-append strBuilder  "\"" tmp "\" ")) )
                        )
                        (if (string-match rightMatch x)
                                (begin (set! flag 0) (set! strBuilder (string-append strBuilder ") ")))
                               	(set! flag 1)
                                ;;(set! intCounter 0)
                	)
		)
	) ;; END my-interpret-group 


	(set! strRule (string-trim strRule))
	(if (string-match "^#" strRule) (display "exit\n") (begin  ; exit if there is a comment # at start of string

	(set! strBuilder (string-append strBuilder "(chat-rule\n	'("))
	
	;; sentense level processing
#!
	(if (string-match "^.*[[:space:]]<[[:space:]].*$" strRule) (begin ; start-with <
        	(set! tmp strRule)
		(set! tmp (string-trim (regexp-substitute #f (string-match "^.*<" tmp)  'pre "" 'post) ) ) ; discard everything before the '<'
                (set! tmp (string-trim (regexp-substitute #f (string-match "[[:space:]]*" (string-trim tmp))  'pre "" 'post) ) )
		(set! strBuilder (string-append strBuilder "(start-with \"" tmp "\"" ") " ) )
		;(display (string-append "\n\n" strBuilder "\n\n"))
		;(set! strBuilder (string-append strBuilder (my-interpret x) )
        ))
		
!#		

	;(set! strBuilder (string-append strBuilder (string-concatenate (map my-interpret (string-split strRule #\space)) ) ) )
	
	;(for-each (lambda (x) 	(set! strBuilder (string-append strBuilder (string-concatenate (map my-interpret x)) )  
	(for-each (lambda (x)  
		(set! x (string-trim x))
                (set! tmp (string-trim (string-filter (lambda (c) (not (or (char=? #\[ c) (char=? #\] c) ))) x)))	
		(cond
                                ;; or-choices
				( (or (string-match "\\[" x) (equal? flag_or-choice 1) ) 
					
					(if (equal? intCounter 0) 
						(begin
							(set! strBuilder (string-append strBuilder "(or-choices "))
							(if (not (equal? tmp "")) (set! strBuilder (string-append strBuilder "\""  tmp "\" ")) )
							(set! intCounter 1)
						)
						(if (not (equal? tmp ""))  (set! strBuilder (string-append strBuilder  "\"" tmp "\" ")))	
					)
					(if (string-match "\\]" x)
                                                (begin (set! flag_or-choice 0) (set! strBuilder (string-append strBuilder ") ")))
                                                (set! flag_or-choice 1)
                                        )
				) ;; END or-choices
				
				
				;; proper-names
	;	#!
                                ( (or (string-match "^''" x) (equal? flag_proper-names 1) )
					;(set! tmp (string-trim (string-filter (lambda (c) (not  (char=? #\' c) )) x)))
					(if (string-match "''" tmp) (set! tmp (regexp-substitute #f (string-match "''" tmp)  'pre "" 'post) ) )
						
                                        (if (equal? intCounter 0)
                                                (begin
                                                        (set! strBuilder (string-append strBuilder "(proper-names "))
                                                        (if (equal? tmp "") (display "") (set! strBuilder (string-append strBuilder "\""  tmp "\" ")) )
                                                        (set! intCounter 1)
                                                )
                                                (if (not (equal? tmp ""))  (set! strBuilder (string-append strBuilder  "\"" tmp "\" ")) )
                                        )
                                        (if (string-match "['']$" x)
                                                (begin (set! flag_proper-names 0) (set! strBuilder (string-append strBuilder ") ")))
                                                (set! flag_proper-names 1)
						;;(set! intCounter 0)
                                        )
                                ) ;; END proper-names
	;	!#		
				;(my-interpret-group x "proper-names" "''" "''" flag_proper-names) ;leftMatch rightMatch flag)
					
				;; unordered matches
				( (or (string-match "<<" x) (equal? flag_unordered-matches 1) )
                                        (if (string-match "<<" tmp) (set! tmp (regexp-substitute #f (string-match "<<" tmp)  'pre "" 'post) ) )
					(if (string-match ">>" tmp) (set! tmp (regexp-substitute #f (string-match ">>" tmp)  'pre "" 'post) ) )
                                        ;(display  intCounter2)
					(if (equal? intCounter2 0)
                                                (begin
                                                        (set! strBuilder (string-append strBuilder "(unordered-matches "))
                                                        (if (equal? tmp "") (display "") (set! strBuilder (string-append strBuilder "\""  tmp "\" ")) )
                                                        (set! intCounter2 1)
                                                )
                                                (if (not (equal? tmp ""))  (set! strBuilder (string-append strBuilder  "\"" tmp "\" ")) )
                                        )
                                        (if (string-match ">>" x)
                                                (begin (set! flag_unordered-matches 0) (set! strBuilder (string-append strBuilder ") ")))
                                                (set! flag_unordered-matches 1)
						;;(set! intCounter 0)
                                        )
                                ) ;; END unordered matches


						
				;; Ignore everything after # because it's a comment
				;( (string-match "#" x) 
					;; how to break out of a for-each?  
				;)
                               	 
				;; single word processing
                                (else ( if (equal? x "") (display "") (set! strBuilder (string-append strBuilder (my-interpret x) ) )))
                )



	)(string-split strRule #\space))
	
	; startwith alternate code
	;(if (string-match "^.*[[:space:]]<[[:space:]].*$" strRule) (begin ; start-with <
        ;        (set! tmp strRule)
        ;        (set! tmp (string-trim (regexp-substitute #f (string-match "^.*<" tmp)  'pre "" 'post) ) ) ; discard everything before the '<'
        ;        (set! tmp (string-trim (regexp-substitute #f (string-match "[[:space:]]*" (string-trim tmp))  'pre "" 'post) ) )
        ;        (set! strBuilder (string-append strBuilder "(start-with \"" tmp "\"" ") " ) )
        ;        ;(display (string-append "\n\n" strBuilder "\n\n"))
        ;        ;(set! strBuilder (string-append strBuilder (my-interpret x) )
        ;))


	(set! strBuilder (string-append strBuilder ")\n") )
	(set! strBuilder (string-append strBuilder "	'(say \"" strSay "\"))" ) )
	(display strBuilder)
	(newline)
	;(eval-string strBuilder)
	)) ;; END exit if there is a comment # at start of string
)
