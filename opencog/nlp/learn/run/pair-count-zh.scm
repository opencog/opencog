;
; pair-count-yue.scm
;
; Run the cogserver, needed for the language-learning word-pair
; counting pipeline. Starts the cogserver, opens the database.
;
(use-modules (opencog) (opencog cogserver))
(use-modules (opencog persist) (opencog persist-sql))
(use-modules (opencog nlp) (opencog nlp learn))

; Tell opencog where the relex server is located.
; The port should match that in `relex-server-any.sh`
(use-relex-server "127.0.0.1" 4445)

; Start the cogserver.
; Edit the below, setting it to the desired langauge.
; This has almost no effect, other than to set the cogserver
; port-number and the prompt-style.
(start-cogserver "opencog-zh.conf")

; Open the database.
; Edit the below, setting the database name, user and password.
(sql-open "postgres:///zh_pairs?user=ubuntu&password=asdf")
