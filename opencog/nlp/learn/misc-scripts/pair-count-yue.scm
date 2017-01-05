;
; pair-count-yue.scm
;
; Run the cogserver, needed for the language-learning word-pair
; counting pipeline. Starts the cogserver, opens the database.
;
(use-modules (opencog) (opencog cogserver))
(use-modules (opencog persist) (opencog persist-sql))

; Start the cogserver.
; Edit the below, setting it to the desired langauge.
; This has almost no effect, other than to set the cogserver
; port-number and the prompt-style.
(start-cogserver "opencog-yue.conf")

; Open the database.
; Edit the below, setting the database name, user and password.
(sql-open "yue_pairs" "ubuntu" "asdf")
