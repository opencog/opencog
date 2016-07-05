(define file_dir (dirname (current-filename)))

(display "Loading 'pickup.scm'\n")
(primitive-load (string-append file_dir "/pickup.scm"))

(display "Loading 'generic.scm'\n")
(primitive-load (string-append file_dir "/generic.scm"))

(display "Loading 'futurist.scm'\n")
(primitive-load (string-append file_dir "/futurist.scm"))
