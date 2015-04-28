;
; dbi-example.scm
;
; Example code for using guile-dbi to access an SQL database, and
; use it with opencog. Pre-requisites: you must have guile-dbi installed,
; you must have a database you want to connect to, etc.
;
; 
(use-modules (dbi dbi))
;
; username is 'linas' in this example, passwd is 'asdf' and the
; databasse is 'lexat', running on the local machine
; The postgres server is at port 5432, which can be gotten from 
; /etc/postgresql/8.3/main/postgresql.conf
; ;
(define my-connect 
	(dbi-open "postgresql" "linas:asdf:lexat:tcp:localhost:5432"))
;
; display the connect status
;
(display my-connect) (newline)
;
; Alternately, one can use a unix-domain socket, located in
; /var/run/postgresql
; (define my-connect 
;      (dbi-open "postgresql" "linas:asdf:lexat:socket:/var/run/postgresql"))
;
; Perform the datbase query
(dbi-query my-connect "SELECT * FROM pairs WHERE left_word='motorcycle'")
;
; Display the query status
;
(display my-connect) (newline)
;
; Print each row inside a loop
; Each row will be an associative array
;
(define row #f)
(set! row (dbi-get_row my-connect))
(while (not (equal? row #f))
	(display row) (newline)
	(set! row (dbi-get_row my-connect))
)

