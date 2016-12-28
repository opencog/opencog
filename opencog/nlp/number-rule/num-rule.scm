; Creates a NumberNode and associate a ReferenceLink to the WordInstance
(define-public (num-rule sent)
	(define word-lists (cdr (car (sent-get-words-in-order sent))))
	(define prep-list (make-vector l))
	(define count 0)
	(define input "")
	(define len 0)
	(define createReference #f)

; Extracts the word from the word instance
(define (process_word_item word)
	(define word_item (string-trim-right (string-trim-right (cog-name word) (lambda (x) (not (eqv? #\@ x)))) #\@))
;		(vector-set! trimmed_lists count word_item)
		(validate word_item)
		(if (eqv? #t isValidInput)
	     	(begin
	     		(vector-set! prep-list count word_item)
	     		(set! len (+ 1 len))
	     		(set! createReference #t))

	     	(if createReference 
	     			(begin 
	     				(mReferenceLink)
	     				(set! len 0)))
		)
		(set! count (+ count 1)))

	

(walk-list word-lists process_word_item)
(set! count 0)

; trimmed_lists contains list of words in sent, 
; prep-list contains number inputs,
; valid-str-index contains the index of the valid inputs from the word-lists.

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
;	(begin (vector-set! v count (list-ref word-lists x)) (set! count (+ count 1)) ) )
;) 
;(array->list valid-str-index))

;(ReferenceLink
;	(NumberNode (strtonum input)) ; Converts input to NumberNode and create ReferenceLink
;	(ListLink
;		(array->list v)
;	))
;)

(define (mReferenceLink)
	(set! createReference #f)
	(define i 0)
	(define mListLink (make-vector len))

	if ((> len 0)
		(begin
			(set! input (string-append input " " vector-ref prep-list i))
			(vector-set! mListLink i (list-ref word-lists (- (+ count 1) len)))
			(set! i (+ 1 i))
			(set! len (- 1 len))
		))
	
	(define number (strtonum input))

	(ReferenceLink
		(NumberNode number) ; Converts input to NumberNode and create ReferenceLink
		(ListLink
			(array->list mListLink))))