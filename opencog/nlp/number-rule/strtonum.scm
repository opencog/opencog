; This scheme procedure convers strings like "two hundred and twenty five" into
; numbers like 225.0 that can be used in numerical calculations. 
;
; It also handles strings that are already numbers like "225" -> 225
;
; It cleans the input string provided by removing leading and trailing white spaces, 
; removing the word "and", and removing the character  '-'. 
; 
; It also changes every character into lower case.
;
; You can acess the procedure as follows:
;
; (strtonum "string")
; 
; example:
;
; -> (strtonum "Two hundred twenty-five thousand six hundred and one")
; -> 225601.0
; 
; -> (strtonum "12345")
; -> 12345
;

(define (strtonum input)
  (if (eqv? #f (string->number input)) ; if string is not in number form. i,e "123" 
      (string->number-words input)
      (string->number input)))

(define continueLoop #t)

(define (string->number-words input)
(define result 0)
(define finalResult 0.0)

; if(input != null && input.length()> 0)
(define (check-null input)
  (cond ((and 
          (not (null? input)) 
          (> (string-length input) 0))
         #t)
        (else #f)))
  
  
  ; converts string to number
(define (calc-result string) 
  (cond ((equal? string "zero") (set! result (+ result 0)))
        ((equal? "one" string) (set! result (+ result 1)))
        ((equal? string "two") (set! result (+ result 2)))
        ((equal? string "three") (set! result (+ result 3)))
        ((equal? string "four") (set! result (+ result 4)))
        ((equal? string "five") (set! result (+ result 5)))
        ((equal? string "six") (set! result (+ result 6)))
        ((equal? string "seven") (set! result (+ result 7)))
        ((equal? string "eight") (set! result (+ result 8)))
        ((equal? string "nine") (set! result (+ result 9)))
        ((equal? string "ten") (set! result (+ result 10)))
        ((equal? string "eleven") (set! result (+ result 11)))
        ((equal? string "twelve") (set! result (+ result 12)))
        ((equal? string "thirteen") (set! result (+ result 13)))
        ((equal? string "fourteen") (set! result (+ result 14)))
        ((equal? string "fifteen") (set! result (+ result 15)))
        ((equal? string "sixteen") (set! result (+ result 16)))
        ((equal? string "seventeen") (set! result (+ result 17)))
        ((equal? string "eighteen") (set! result (+ result 18)))
        ((equal? string "nineteen") (set! result (+ result 19)))
        ((equal? string "twenty") (set! result (+ result 20)))
        ((equal? string "thirty") (set! result (+ result 30)))
        ((equal? string "forty") (set! result (+ result 40)))
        ((equal? string "fifty") (set! result (+ result 50)))
        ((equal? string "sixty") (set! result (+ result 60)))
        ((equal? string "seventy") (set! result (+ result 70)))
        ((equal? string "eighty") (set! result (+ result 80)))
        ((equal? string "ninety") (set! result (+ result 90)))
        ((equal? string "hundred") (set! result (* result 100)))
        ((equal? string "thousand") (set! result (* result 1000))
                                    (set! finalResult (+ result finalResult))
                                    (set! result 0.0))
         ((equal? string "million") (set! result (* result 1000000))
                                    (set! finalResult (+ result finalResult))
                                    (set! result 0.0))
         ((equal? string "billion") (set! result (* result 1000000000))
                                    (set! finalResult (+ result finalResult))
                                    (set! result 0.0))
         ((equal? string "trillion") (set! result (* result 1000000000000))
                                    (set! finalResult (+ result finalResult))
                                    (set! result 0.0))
        ))

; if input != null or not empty
(cond ((check-null input) (begin 
                            (set! input (string-downcase input))             ; convert to lower case
                            (set! input (string-replace-char input #\- #\ )) ; replace '-' with space ' '
                            (set! input (string-trim-both input))            ; omit leading and trailing whitespace
                            (set! input (string-tokenize input))             ; create a list tokenized by white space
                            (set! input (delete "and" input))                ; delete the "and" string from input list
                            ;(set! input (remove* (list "and") input))        
                            (walk-list input validate-string)                ; validate input
                            (walk-list input calc-result)                    ; covert input string
                            (set! finalResult (+ result finalResult))
                            finalResult))))                                  
;------
;(set! input (delete "and" input)) ;You should use this instead in Guile
;--------

  ; Iterates list applying fun to each item
(define (walk-list lst fun)
   (if (not (list? lst))
      (fun lst)
      (cond ((not (null? lst))
         (begin
            (walk-list (car lst) fun)
            (walk-list (cdr lst) fun))))))

;Needed for checking validation
(define isValidInput #t)

; replace every instance of a character with another
(define (string-replace-char input findChar replaceWith)
  (define index #f)
  (set! input (substring input 0)) ; change to mutable string
  (set! index (string-index input (lambda (x) 
                                    (eqv? x findChar))))
  (cond ((not (eqv? index #f)) (string-set! input index replaceWith) 
                               (string-replace-char input findChar replaceWith))
        (else input)))

(define (validate input)
    (set! input (string-downcase input))             ; convert to lower case
    (set! input (string-replace-char input #\- #\ )) ; replace '-' with space ' '
    (set! input (string-trim-both input))            ; omit leading and trailing whitespace
    (set! input (string-replace-char input #\] #\ )) ; replace ']' with space ' '
    (set! input (string-replace-char input #\[ #\ )) ; replace '[' with space ' '
    (set! input (string-tokenize input))             ; create a list tokenized by white space
    (set! input (delete "and" input))                ; delete the "and" string from input list

    (walk-list input validate-string))                ; validate input

; check if input is in allowedStrings list
(define (validate-string input)

  (define allowedStrings (list "zero" "one" "two" "three" "four" "five" "six" "seven" "eight" 
                             "nine" "ten" "eleven" "twelve" "thirteen" "fourteen" "fifteen"
                             "sixteen" "seventeen" "eighteen" "nineteen" "twenty" "thirty" 
                             "forty" "fifty" "sixty" "seventy" "eighty" "ninety"  "hundred" 
                             "thousand" "million" "billion" "trillion" "and"))

  
  (if (and (eqv? #f (member input allowedStrings)) (eqv? #f (string->number input)))
      (begin (set! isValidInput #f) 
             (set! continueLoop #f))
        (set! isValidInput #t)))

