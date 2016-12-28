; Creates a NumberNode and associate a ReferenceLink to the WordInstance
(define-public (num-rule sent)
	(define word_lists (cdr (car (sent-get-words-in-order sent))))
	(define l (length word_lists))
	(define prep-list (make-vector l))
	(define count 0)
	(define input "")
	(define len 0)
	(define createReference #f)

(define (process_word_item word)
	(define word_item (string-trim-right (string-trim-right (cog-name word) (lambda (x) (not (eqv? #\@ x)))) #\@))
		(validate word_item)
		;(display word_item)
		;(newline)
		(if (eqv? #t isValidInput)
	     	(begin
	     		(vector-set! prep-list len word_item)
	     		(set! len (+ 1 len))
	     		(set! createReference #t))
	     	(begin
	     		(if createReference 
	     			(begin 
	     				(display (mReferenceLink))
	     				(set! len 0)))))
		(set! count (+ count 1))
		)

(define (mReferenceLink)
	(define i 0)
	(define mListLink (make-vector len))
	(define number 0)
	(set! createReference #f)
	(if (> len 0)
		(begin
			(set! input (string-append input " " (vector-ref prep-list i)))
			(vector-set! mListLink i (list-ref word_lists (- count len)))
			(set! i (+ 1 i))
			(set! len (- 1 len))
		))

	(set! number (strtonum input)) ; Converts input to NumberNode and create ReferenceLink

	(ReferenceLink
		(NumberNode number) 
		(ListLink
			(array->list mListLink))))

(walk-list word_lists process_word_item))

; Extracts the word from the word instance


;(set! count 0)

; trimmed_lists contains list of words in sent, 
; prep-list contains number inputs,
; valid-str-index contains the index of the valid inputs from the word_lists.

;(for-each (lambda (x)
;	(if (number? x) (set! x (number->string x)))
; 	(set! x (string-downcase x))             ; convert to lower case
; 	(set! x (string-replace-char x #\] #\ )) ; replace ']' with space ' '
; 	(set! x (string-replace-char x #\[ #\ )) ; replace '[' with space ' '
; 	(set! x (string-trim-both x))            ; omit leading and trailing whitespace
; 	(validate-string x)
;	(if (number? (string->number x)) (set! isValidInput #t))
; 	(if (eqv? #t isValidInput)
;     	(begin
;     	(vector-set! prep-list count x)
;     	(vector-set! valid-str-index count count))
;    	 (begin 
;     	(vector-set! prep-list count #f)
;     	(vector-set! valid-str-index count #f)))	
;  	(set! count (+ count 1))
;)
;(array->list trimmed_lists))

;(set! count 0) 
;(for-each (lambda (x) (if (not (eqv? #f x))
;			(begin
;				(if (number? x) (set! x (number->string x)))
;				(set! input (string-append input " " x))
;				(set! count (+ count 1))) )
;)
;(array->list prep-list))

; v contains the word instances the NumberNode is referring
;(set! v (make-vector count))
;(set! count 0)
;(for-each (lambda (x) (if (not (eqv? #f x)) 
;	(begin (vector-set! v count (list-ref word_lists x)) (set! count (+ count 1)) ) )
;) 
;(array->list valid-str-index))

;(ReferenceLink
;	(NumberNode (strtonum input)) ; Converts input to NumberNode and create ReferenceLink
;	(ListLink
;		(array->list v)
;	))
;)