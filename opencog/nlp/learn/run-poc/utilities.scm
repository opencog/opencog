; Used to specify the command-line arguments for getting language and
; database details.
; Depending on your database configuration, the usage patterns for scripts
; that invoke this function are,
   
; guile -l script.scm -- --mode pairs --lang en --db wiki --user opencog_user --password cheese
; or
; guile -l script.scm -- --mode pairs --lang en --db wiki
  
(use-modules (ice-9 getopt-long))

(define option-spec
  '((mode (required? #t) (value #t))
  	(lang (required? #t) (value #t))
    (db (required? #t) (value #t))
    (user (required? #f) (value #t))
    (password (required? #f) (value #t)))
)

(define options (getopt-long (command-line) option-spec))

(define pw (option-ref options 'password ""))
(define db_user (option-ref options 'user ""))

(define (get-mode) 
  (option-ref options 'mode #f))

(define (get-lang) 
  (option-ref options 'lang #f))

(define (get-connection-uri)
  (string-append
      "postgres:///" (option-ref options 'db #f)
      (if (equal? "" db_user) "" (format #f "?user=~a" db_user))
      (if (equal? "" pw) "" (format #f "&password=~a" pw)))
)