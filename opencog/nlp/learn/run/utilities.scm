(use-modules (ice-9 getopt-long))

(define (get-connection-uri)
"
  Used to specify the command-line arguments for getting database details.
  Depending on your database configuration, the usage patterns for scripts
  that invoke this function are,

    guile -l script.scm -- --db wiki --user opencog_user --password cheese
  or
    guile -l script.scm -- --db wiki --user opencog_user

"
  ;TODO This assumes local connection update for networked access.
  (let* ((option-spec
            '((db (required? #t) (value #t))
              (user (required? #f) (value #t))
              (password (required? #f) (value #t))))
        (options (getopt-long (command-line) option-spec))
        (pw (option-ref options 'password ""))
        (db_user (option-ref options 'user "")))

    (string-append
      "postgres:///" (option-ref options 'db #f)
      (if (equal? "" db_user) "" (format #f "?user=~a" db_user))
      (if (equal? "" pw) "" (format #f "&password=~a" pw)))
  )
)
