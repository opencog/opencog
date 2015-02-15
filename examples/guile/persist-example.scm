;
; persist-example.scm
;
; Linas Vepstas February 2015
;
; This file contains some demo coe for using the SQL persistance 
; database. It explores database login, logout and atom loading and
; storage.  It assumes that the SQL database has been configured
; correctly, per the usual instructions.
;
; This example can be run in two different ways.  The first way requires
; that the cogserver be running.  Telnet to either port 17001 or 18001:
;    telnet localhost 17001
; and then cut-n-paste contents from this file.
;
; The second way of running this is to run it without the cogserver,
; using only guile:
;
;    export LTDL_LIBRARY_PATH=build/opencog/guile:build/opencog/query:build/opencog/persist/guile:build/opencog/persist/sql
;    cd ../..
;    guile -L build -L opencog/scm
;
;    (add-to-load-path "/home/yourname/opencog-git/build")
;    (add-to-load-path "/home/yourname/opencog-git/opencog/scm")
;

(use-modules (ice-9 readline))
(activate-readline)

(use-modules (opencog))
(use-modules (opencog persist))
(use-modules (opencog persist-sql))
(load-from-path "utilities.scm")

; The below should throw an exception, since the database is not yet
; open.
(store-atom (ConceptNode "asdf" (stv 0.42 0.24)))

; Use the test database credentials. You should probably user your own,
; and not these, sice the unit tests will wipe this out.
(sql-open "opencog_test" "opencog_tester" "cheese")

; Try storing again
(store-atom (ConceptNode "asdf" (stv 0.318309886 0.36787944)))

; Close the database.
(sql-close)

; Try fetching the atom. This should fail!
(fetch-atom (ConceptNode "asdf"))

; Reopen the database
(sql-open "opencog_test" "opencog_tester" "cheese")

; Try fetching the atom. This time it should work.  Notice that
; it retreived the correct truth value.
(fetch-atom (ConceptNode "asdf"))

; Change it's truth value, store it, and repeat.
(define my-atom (ConceptNode "asdf"))
(cog-set-tv! my-atom (stv 0.25 0.75))
(store-atom my-atom)
(sql-close)
(sql-open "opencog_test" "opencog_tester" "cheese")
(fetch-atom (ConceptNode "asdf"))

; That's all for now
